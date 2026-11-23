// 变换存储和它的 C API。世界矩阵用 Jolt 的 DMat44（旋转缩放单精度、平移双精度）算，按父节点在前的顺序一遍算完。
#include <Core/Core.h>
#include <Core/Debug.h>
#include <Core/HandleTable.h>
#include <Runtime/Transforms.h>
#include <StrideNative/Runtime.h>

#include <algorithm>
#include <cmath>
#include <mutex>

using namespace JPH;

namespace StrideNative
{
	namespace
	{
		/// 一层里的槽位至少这么多才分块并行（再少的话调度的开销比算的还大）
		constexpr uint32_t cParallelLevelSize = 8192;

		/// 列主序的 4x4 双精度矩阵（JS 读的布局）和 DMat44 互换
		void StoreDMat44(DMat44Arg inMatrix, double *outValues)
		{
			for (uint col = 0; col < 3; ++col)
			{
				Vec4 c = inMatrix.GetColumn4(col);
				outValues[col * 4] = c.GetX();
				outValues[col * 4 + 1] = c.GetY();
				outValues[col * 4 + 2] = c.GetZ();
				outValues[col * 4 + 3] = 0.0;
			}
			DVec3 t = inMatrix.GetTranslation();
			outValues[12] = t.GetX();
			outValues[13] = t.GetY();
			outValues[14] = t.GetZ();
			outValues[15] = 1.0;
		}

		DMat44 LoadDMat44(const double *inValues)
		{
			auto column = [inValues](uint inCol) { return Vec4(float(inValues[inCol * 4]), float(inValues[inCol * 4 + 1]), float(inValues[inCol * 4 + 2]), 0.0f); };
			return DMat44(column(0), column(1), column(2), DVec3(inValues[12], inValues[13], inValues[14]));
		}
	}

	TransformStore::TransformStore(uint32_t inCapacity) :
		mPosition(size_t(inCapacity + 1) * 3, 0.0),
		mRotation(size_t(inCapacity + 1) * 4, 0.0f),
		mScale(size_t(inCapacity + 1) * 3, 1.0f),
		mParent(inCapacity + 1, 0),
		mWorld(size_t(inCapacity + 1) * 16, 0.0),
		mRenderWorld(size_t(inCapacity + 1) * 16, 0.0f),
		mCapacity(inCapacity),
		mAlive(inCapacity + 1, 0),
		mChildCount(inCapacity + 1, 0),
		mLinkSlot(inCapacity + 1, 0),
		mLinkMatrix(size_t(inCapacity + 1) * 16, 0.0f),
		mChangedStamp(inCapacity + 1, 0)
	{
		// 只有槽位 0（"没有父节点"）要现成的单位矩阵；别的槽位在 Allocate 里初始化，这里不碰它们的内存
		mRotation[3] = 1.0f;
		StoreDMat44(DMat44::sIdentity(), &mWorld[0]);
		Mat44::sIdentity().StoreFloat4x4(reinterpret_cast<Float4 *>(&mRenderWorld[0]));
	}

	void TransformStore::Grow(uint32_t inCapacity)
	{
		const size_t slots = size_t(inCapacity) + 1;
		mPosition.resize(slots * 3);
		mRotation.resize(slots * 4);
		mScale.resize(slots * 3);
		mParent.resize(slots);
		mWorld.resize(slots * 16);
		mRenderWorld.resize(slots * 16);
		mAlive.resize(slots);
		mChildCount.resize(slots);
		mLinkSlot.resize(slots);
		mLinkMatrix.resize(slots * 16);
		mChangedStamp.resize(slots);
		mCapacity = inCapacity;
		++mGeneration;
	}

	bool TransformStore::Allocate(int32_t *outSlots, uint32_t inCount)
	{
		if (inCount > mFreeSlots.size() + (mCapacity - mHighWater))
		{
			// 增长到够用：至少翻倍（几十万个实体一个个建时，拷贝的总量是 O(n)）
			uint64_t needed = uint64_t(mHighWater) + inCount - mFreeSlots.size();
			if (needed > cMaxCapacity)
				return false;
			Grow(uint32_t(std::min<uint64_t>(std::max<uint64_t>(needed, uint64_t(mCapacity) * 2), cMaxCapacity)));
		}
		for (uint32_t i = 0; i < inCount; ++i)
		{
			int32_t slot;
			if (!mFreeSlots.empty())
			{
				slot = mFreeSlots.back();
				mFreeSlots.pop_back();
			}
			else
				slot = int32_t(++mHighWater);

			std::fill_n(&mPosition[slot * 3], 3, 0.0);
			std::fill_n(&mRotation[slot * 4], 3, 0.0f);
			mRotation[slot * 4 + 3] = 1.0f;
			std::fill_n(&mScale[slot * 3], 3, 1.0f);
			mParent[slot] = 0;
			mChildCount[slot] = 0;
			mLinkSlot[slot] = 0;
			StoreDMat44(DMat44::sIdentity(), &mWorld[slot * 16]);
			StoreRenderWorld(slot);
			mAlive[slot] = 1;
			MarkChanged(slot);
			outSlots[i] = slot;
		}
		mOrderDirty |= inCount > 0;
		return true;
	}

	bool TransformStore::Free(const int32_t *inSlots, uint32_t inCount, std::string &outError)
	{
		// 先把要释放的槽位标成 2，确认每个都活着且不重复
		uint32_t marked = 0;
		for (; marked < inCount; ++marked)
		{
			int32_t slot = inSlots[marked];
			if (!IsAlive(slot) || mAlive[slot] == 2)
			{
				outError = "slot " + std::to_string(slot) + (IsAlive(slot) ? " is listed twice" : " is not allocated");
				break;
			}
			mAlive[slot] = 2;
		}
		// 一起释放的子节点不算：被释放的子节点不再计入父节点
		auto unlink = [&](int32_t inDelta)
		{
			for (uint32_t i = 0; i < marked; ++i)
				if (int32_t parent = mParent[inSlots[i]]; parent != 0)
					mChildCount[parent] += inDelta;
		};
		if (marked == inCount)
		{
			unlink(-1);
			for (uint32_t i = 0; i < inCount && outError.empty(); ++i)
				if (mChildCount[inSlots[i]] != 0)
					outError = "slot " + std::to_string(inSlots[i]) + " still has " + std::to_string(mChildCount[inSlots[i]]) + " child(ren)";
			if (outError.empty())
			{
				for (uint32_t i = 0; i < inCount; ++i)
				{
					mAlive[inSlots[i]] = 0;
					MarkChanged(inSlots[i]);
					mParent[inSlots[i]] = 0;
					mLinkSlot[inSlots[i]] = 0;
					mFreeSlots.push_back(inSlots[i]);
				}
				// 释放的槽位上的链接、指向释放的槽位的链接都去掉（那些槽位变回按父节点算）
				mLinked.erase(std::remove_if(mLinked.begin(), mLinked.end(), [this](int32_t inLinked)
				{
					if (mAlive[inLinked] != 0 && mLinkSlot[inLinked] != 0 && mAlive[mLinkSlot[inLinked]] != 0)
						return false;
					mLinkSlot[inLinked] = 0;
					return true;
				}), mLinked.end());
				mOrderDirty |= inCount > 0;
				return true;
			}
			unlink(+1);
		}
		for (uint32_t i = 0; i < marked; ++i)
			mAlive[inSlots[i]] = 1;
		return false;
	}

	bool TransformStore::SetParent(int32_t inSlot, int32_t inParent, std::string &outError)
	{
		if (!IsAlive(inSlot))
		{
			outError = "slot " + std::to_string(inSlot) + " is not allocated";
			return false;
		}
		if (inParent != 0 && !IsAlive(inParent))
		{
			outError = "parent slot " + std::to_string(inParent) + " is not allocated";
			return false;
		}
		for (int32_t ancestor = inParent; ancestor != 0; ancestor = mParent[ancestor])
			if (ancestor == inSlot)
			{
				outError = "slot " + std::to_string(inSlot) + " cannot become a descendant of itself";
				return false;
			}
		// 没有链接的槽位按父节点算世界矩阵：父节点也不能经由链接依赖它
		if (mLinkSlot[inSlot] == 0)
			for (int32_t ancestor = inParent; ancestor != 0; ancestor = EffectiveParent(ancestor))
				if (ancestor == inSlot)
				{
					outError = "slot " + std::to_string(inSlot) + " cannot become the parent of a slot linked to it";
					return false;
				}

		if (int32_t old = mParent[inSlot]; old != 0)
			--mChildCount[old];
		if (inParent != 0)
			++mChildCount[inParent];
		mParent[inSlot] = inParent;
		mOrderDirty = true;
		return true;
	}

	bool TransformStore::SetLink(int32_t inSlot, int32_t inLinkSlot, const float *inMatrix, std::string &outError)
	{
		if (!IsAlive(inSlot))
		{
			outError = "slot " + std::to_string(inSlot) + " is not allocated";
			return false;
		}
		if (inLinkSlot != 0)
		{
			if (!IsAlive(inLinkSlot))
			{
				outError = "link slot " + std::to_string(inLinkSlot) + " is not allocated";
				return false;
			}
			if (inMatrix == nullptr)
			{
				outError = "a link needs a matrix";
				return false;
			}
			// 链接的槽位不能依赖这个槽位（沿算世界矩阵的"父"往上走不能走到自己）
			for (int32_t ancestor = inLinkSlot; ancestor != 0; ancestor = EffectiveParent(ancestor))
				if (ancestor == inSlot)
				{
					outError = "slot " + std::to_string(inSlot) + " cannot link to its own descendant " + std::to_string(inLinkSlot);
					return false;
				}
			std::copy_n(inMatrix, 16, &mLinkMatrix[inSlot * 16]);
		}
		if (mLinkSlot[inSlot] == inLinkSlot)
			return true;
		if (mLinkSlot[inSlot] == 0)
			mLinked.push_back(inSlot);
		else if (inLinkSlot == 0)
			mLinked.erase(std::find(mLinked.begin(), mLinked.end(), inSlot));
		mLinkSlot[inSlot] = inLinkSlot;
		mOrderDirty = true;
		return true;
	}

	DMat44 TransformStore::ParentWorld(int32_t inSlot) const
	{
		if (int32_t link = mLinkSlot[inSlot]; link != 0)
			return LoadWorld(link) * Mat44::sLoadFloat4x4(reinterpret_cast<const Float4 *>(&mLinkMatrix[inSlot * 16]));
		if (int32_t parent = mParent[inSlot]; parent != 0)
			return LoadWorld(parent);
		return DMat44::sIdentity();
	}

	void TransformStore::RebuildOrder()
	{
		// 每个槽位的深度：沿父链往上走到已知深度的祖先（或根），再往回填，整体是 O(n)
		mDepth.assign(mHighWater + 1, -1);
		int32_t max_depth = -1;
		uint32_t alive = 0;
		for (int32_t slot = 1; slot <= int32_t(mHighWater); ++slot)
		{
			if (mAlive[slot] == 0)
				continue;
			++alive;
			if (mDepth[slot] >= 0)
				continue;
			mChain.clear();
			int32_t known = slot;
			for (; known != 0 && mDepth[known] < 0; known = EffectiveParent(known))
				mChain.push_back(known);
			int32_t depth = known == 0 ? -1 : mDepth[known];
			for (auto it = mChain.rbegin(); it != mChain.rend(); ++it)
				mDepth[*it] = ++depth;
			max_depth = std::max(max_depth, depth);
		}

		// 按深度计数排序：同一深度内保持槽位号顺序，内存访问大体是顺序的
		mBuckets.assign(size_t(max_depth + 2), 0);
		for (int32_t slot = 1; slot <= int32_t(mHighWater); ++slot)
			if (mAlive[slot] != 0)
				++mBuckets[mDepth[slot] + 1];
		for (size_t d = 1; d < mBuckets.size(); ++d)
			mBuckets[d] += mBuckets[d - 1];
		mOrder.resize(alive);
		for (int32_t slot = 1; slot <= int32_t(mHighWater); ++slot)
			if (mAlive[slot] != 0)
				mOrder[mBuckets[mDepth[slot]]++] = slot;
		// 放完以后 mBuckets[d] 是第 d 层的结束位置（开始位置是上一层的结束位置）
		mLevelEnd.assign(mBuckets.begin(), mBuckets.begin() + (max_depth + 1));
		mOrderDirty = false;
	}

	DMat44 TransformStore::LocalMatrix(int32_t inSlot) const
	{
		const double *p = &mPosition[inSlot * 3];
		const float *r = &mRotation[inSlot * 4], *s = &mScale[inSlot * 3];
		// 四元数直接按标准公式变成矩阵（同 Stride 的 Matrix.RotationQuaternion），不要求归一化：场景里手填的旋转常常
		// 不是单位长度，Stride 也不归一化（Jolt 的 sRotation 在调试版里会断言）
		const float x = r[0], y = r[1], z = r[2], w = r[3];
		const float xx = x * x, yy = y * y, zz = z * z, xy = x * y, zw = z * w, zx = z * x, yw = y * w, yz = y * z, xw = x * w;
		Mat44 rotation(
			Vec4(1.0f - 2.0f * (yy + zz), 2.0f * (xy + zw), 2.0f * (zx - yw), 0.0f),
			Vec4(2.0f * (xy - zw), 1.0f - 2.0f * (zz + xx), 2.0f * (yz + xw), 0.0f),
			Vec4(2.0f * (zx + yw), 2.0f * (yz - xw), 1.0f - 2.0f * (yy + xx), 0.0f),
			Vec4(0.0f, 0.0f, 0.0f, 1.0f));
		return DMat44(rotation.PreScaled(Vec3(s[0], s[1], s[2])), DVec3(p[0], p[1], p[2]));
	}

	DMat44 TransformStore::LoadWorld(int32_t inSlot) const
	{
		return LoadDMat44(&mWorld[inSlot * 16]);
	}

	DMat44 TransformStore::ComputeWorldMatrix(int32_t inSlot) const
	{
		DMat44 world = LocalMatrix(inSlot);
		for (int32_t slot = inSlot; EffectiveParent(slot) != 0; slot = EffectiveParent(slot))
		{
			if (mLinkSlot[slot] != 0)
				world = DMat44(Mat44::sLoadFloat4x4(reinterpret_cast<const Float4 *>(&mLinkMatrix[slot * 16]))) * world;
			world = LocalMatrix(EffectiveParent(slot)) * world;
		}
		return world;
	}

	void TransformStore::Update()
	{
		if (mOrderDirty)
			RebuildOrder();
		// 一层一层算（父节点在上一层，已经算完）；一层里的槽位互不相干，层够大时分块并行
		uint32_t begin = 0;
		for (uint32_t end : mLevelEnd)
		{
			if (end - begin < cParallelLevelSize)
			{
				for (uint32_t i = begin; i < end; ++i)
					UpdateOne(mOrder[i], mChanged);
			}
			else
			{
				std::mutex changed_mutex;
				ParallelFor(end - begin, [&](uint32_t inBegin, uint32_t inEnd)
				{
					std::vector<int32_t> changed;
					for (uint32_t i = begin + inBegin; i < begin + inEnd; ++i)
						UpdateOne(mOrder[i], changed);
					std::lock_guard lock(changed_mutex);
					mChanged.insert(mChanged.end(), changed.begin(), changed.end());
				});
			}
			begin = end;
		}
		// 这一轮变了的交出去，开始下一轮
		mPublished.swap(mChanged);
		mChanged.clear();
		++mStamp;
	}

	void TransformStore::UpdateOne(int32_t inSlot, std::vector<int32_t> &ioChanged)
	{
		DMat44 world = EffectiveParent(inSlot) != 0 ? ParentWorld(inSlot) * LocalMatrix(inSlot) : LocalMatrix(inSlot);
		if (!StoreWorld(inSlot, world))
			return;
		// 只碰自己槽位的去重标记，几个线程各算各的槽位时不冲突
		if (mChangedStamp[inSlot] != mStamp)
		{
			mChangedStamp[inSlot] = mStamp;
			ioChanged.push_back(inSlot);
		}
	}

	bool TransformStore::StoreWorld(int32_t inSlot, DMat44Arg inWorld)
	{
		double *stored = &mWorld[inSlot * 16];
		if (LoadDMat44(stored) == inWorld)
			return false;
		StoreDMat44(inWorld, stored);
		StoreRenderWorld(inSlot);
		return true;
	}

	void TransformStore::StoreRenderWorld(int32_t inSlot)
	{
		const double *world = &mWorld[inSlot * 16];
		float *render = &mRenderWorld[inSlot * 16];
		for (int i = 0; i < 12; ++i)
			render[i] = float(world[i]);
		// 先在双精度里减再转单精度：远处的平移减掉渲染原点以后是小数，精度留在相机附近
		render[12] = float(world[12] - mRenderOrigin.GetX());
		render[13] = float(world[13] - mRenderOrigin.GetY());
		render[14] = float(world[14] - mRenderOrigin.GetZ());
		render[15] = 1.0f;
	}

	void TransformStore::SetRenderOrigin(DVec3Arg inOrigin)
	{
		if (inOrigin == mRenderOrigin)
			return;
		mRenderOrigin = inOrigin;
		for (int32_t slot = 1; slot <= int32_t(mHighWater); ++slot)
			if (mAlive[slot] != 0)
			{
				StoreRenderWorld(slot);
				MarkChanged(slot);
			}
	}

	void TransformStore::MarkChanged(int32_t inSlot)
	{
		if (mChangedStamp[inSlot] == mStamp)
			return;
		mChangedStamp[inSlot] = mStamp;
		mChanged.push_back(inSlot);
	}

	void TransformStore::UpdateSlot(int32_t inSlot)
	{
		std::vector<int32_t> chain;
		for (int32_t slot = inSlot; slot != 0; slot = EffectiveParent(slot))
			chain.push_back(slot);
		for (auto it = chain.rbegin(); it != chain.rend(); ++it)
			UpdateOne(*it, mChanged);
	}

	void TransformStore::SetWorldPose(int32_t inSlot, DVec3Arg inPosition, QuatArg inRotation)
	{
		DVec3 position = inPosition;
		Quat rotation = inRotation;
		if (int32_t parent = EffectiveParent(inSlot); parent != 0)
		{
			// 父节点的世界矩阵现算，不用缓存：这一帧脚本可能刚改过父节点（还没 Update），物理写回时父物体也可能刚写过
			DMat44 parent_world = ComputeWorldMatrix(parent);
			if (int32_t link = mLinkSlot[inSlot]; link != 0)
				parent_world = parent_world * Mat44::sLoadFloat4x4(reinterpret_cast<const Float4 *>(&mLinkMatrix[inSlot * 16]));
			Vec3 parent_scale;
			position = parent_world.Inversed() * inPosition;
			rotation = (parent_world.Decompose(parent_scale).GetQuaternion().Conjugated() * inRotation).Normalized();
		}
		mPosition[inSlot * 3] = position.GetX();
		mPosition[inSlot * 3 + 1] = position.GetY();
		mPosition[inSlot * 3 + 2] = position.GetZ();
		rotation.GetXYZW().StoreFloat4(reinterpret_cast<Float4 *>(&mRotation[inSlot * 4]));
	}
}

namespace StrideNative
{
	TransformStore *LookupTransformStore(int32_t inHandle, const char *inWhat)
	{
		return Lookup<TransformStore>(inHandle, inWhat);
	}
}

using namespace StrideNative;

namespace
{
	TransformStore *GetStore(int32_t inHandle)
	{
		return LookupTransformStore(inHandle);
	}
}

SN_API int32_t sn_transforms_create(uint32_t capacity)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (capacity == 0 || capacity > INT32_MAX - 1)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "capacity must be in [1, 2^31 - 2]");
	return GetTable<TransformStore>().Add(std::make_unique<TransformStore>(capacity));
}

SN_API void sn_transforms_destroy(int32_t transforms)
{
	GetTable<TransformStore>().Remove(transforms);
}

SN_API int32_t sn_transforms_generation(int32_t transforms)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	return int32_t(store->Generation());
}

SN_API int32_t sn_transforms_columns(int32_t transforms, sn_transform_columns *out_columns)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (out_columns == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "out_columns is required");
	*out_columns = { store->mPosition.data(), store->mRotation.data(), store->mScale.data(), store->mParent.data(), store->mWorld.data(), store->mRenderWorld.data(), store->Capacity() };
	return SN_OK;
}

SN_API int32_t sn_transforms_alloc(int32_t transforms, int32_t *out_slots, uint32_t count)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (out_slots == nullptr && count > 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "out_slots is required");
	if (!store->Allocate(out_slots, count))
		return SetLastError(SN_ERROR_FAILED, "transform capacity " + std::to_string(store->Capacity()) + " reached");
	return int32_t(count);
}

SN_API int32_t sn_transforms_free(int32_t transforms, const int32_t *slots, uint32_t count)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (slots == nullptr && count > 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "slots are required");
	std::string error;
	return store->Free(slots, count, error) ? SN_OK : SetLastError(SN_ERROR_INVALID_ARGUMENT, error);
}

SN_API int32_t sn_transforms_set_parent(int32_t transforms, int32_t slot, int32_t parent)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	std::string error;
	return store->SetParent(slot, parent, error) ? SN_OK : SetLastError(SN_ERROR_INVALID_ARGUMENT, error);
}

SN_API int32_t sn_transforms_set_link(int32_t transforms, int32_t slot, int32_t link_slot, const float *matrix)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	std::string error;
	return store->SetLink(slot, link_slot, matrix, error) ? SN_OK : SetLastError(SN_ERROR_INVALID_ARGUMENT, error);
}

SN_API int32_t sn_transforms_set_render_origin(int32_t transforms, double x, double y, double z)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "the render origin must be finite");
	store->SetRenderOrigin(DVec3(x, y, z));
	return SN_OK;
}

SN_API int32_t sn_transforms_slot_limit(int32_t transforms)
{
	TransformStore *store = GetStore(transforms);
	return store != nullptr ? int32_t(store->SlotLimit()) : SN_ERROR_INVALID_HANDLE;
}

SN_API int32_t sn_transforms_changed(int32_t transforms, const int32_t **out_slots)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (out_slots == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "out_slots is required");
	*out_slots = store->Changed().data();
	return int32_t(store->Changed().size());
}

SN_API int32_t sn_transforms_update_slot(int32_t transforms, int32_t slot)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (!store->IsAlive(slot))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "transform slot " + std::to_string(slot) + " is not allocated");
	store->UpdateSlot(slot);
	return SN_OK;
}

SN_API int32_t sn_transforms_update(int32_t transforms)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	SN_DEBUG_SCOPE("transforms.update");
	store->Update();
	if (DebugEnabled())
		sn_debug_set_counter("transforms.slotHighWater", store->SlotLimit());
	return SN_OK;
}
