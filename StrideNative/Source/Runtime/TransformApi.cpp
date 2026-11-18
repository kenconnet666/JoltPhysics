// 变换存储和它的 C API。世界矩阵用 Jolt 的 Mat44（SIMD）算，按父节点在前的顺序一遍算完。
#include <Core/Debug.h>
#include <Core/HandleTable.h>
#include <Runtime/Transforms.h>
#include <StrideNative/Runtime.h>

#include <algorithm>

using namespace JPH;

namespace StrideNative
{
	TransformStore::TransformStore(uint32_t inCapacity) :
		mPosition(size_t(inCapacity + 1) * 3, 0.0f),
		mRotation(size_t(inCapacity + 1) * 4, 0.0f),
		mScale(size_t(inCapacity + 1) * 3, 1.0f),
		mParent(inCapacity + 1, 0),
		mWorld(size_t(inCapacity + 1) * 16, 0.0f),
		mCapacity(inCapacity),
		mAlive(inCapacity + 1, 0),
		mChildCount(inCapacity + 1, 0)
	{
		for (uint32_t slot = 0; slot <= inCapacity; ++slot)
		{
			mRotation[slot * 4 + 3] = 1.0f;
			Mat44::sIdentity().StoreFloat4x4(reinterpret_cast<Float4 *>(&mWorld[slot * 16]));
		}
	}

	bool TransformStore::Allocate(int32_t *outSlots, uint32_t inCount)
	{
		if (inCount > mFreeSlots.size() + (mCapacity - mHighWater))
			return false;
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

			std::fill_n(&mPosition[slot * 3], 3, 0.0f);
			std::fill_n(&mRotation[slot * 4], 3, 0.0f);
			mRotation[slot * 4 + 3] = 1.0f;
			std::fill_n(&mScale[slot * 3], 3, 1.0f);
			mParent[slot] = 0;
			mChildCount[slot] = 0;
			Mat44::sIdentity().StoreFloat4x4(reinterpret_cast<Float4 *>(&mWorld[slot * 16]));
			mAlive[slot] = 1;
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
					mParent[inSlots[i]] = 0;
					mFreeSlots.push_back(inSlots[i]);
				}
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

		if (int32_t old = mParent[inSlot]; old != 0)
			--mChildCount[old];
		if (inParent != 0)
			++mChildCount[inParent];
		mParent[inSlot] = inParent;
		mOrderDirty = true;
		return true;
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
			for (; known != 0 && mDepth[known] < 0; known = mParent[known])
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
		mOrderDirty = false;
	}

	Mat44 TransformStore::LocalMatrix(int32_t inSlot) const
	{
		const float *p = &mPosition[inSlot * 3], *r = &mRotation[inSlot * 4], *s = &mScale[inSlot * 3];
		return Mat44::sRotationTranslation(Quat(r[0], r[1], r[2], r[3]), Vec3(p[0], p[1], p[2])).PreScaled(Vec3(s[0], s[1], s[2]));
	}

	Mat44 TransformStore::LoadWorld(int32_t inSlot) const
	{
		return Mat44::sLoadFloat4x4(reinterpret_cast<const Float4 *>(&mWorld[inSlot * 16]));
	}

	Mat44 TransformStore::ComputeWorldMatrix(int32_t inSlot) const
	{
		Mat44 world = LocalMatrix(inSlot);
		for (int32_t parent = mParent[inSlot]; parent != 0; parent = mParent[parent])
			world = LocalMatrix(parent) * world;
		return world;
	}

	void TransformStore::Update()
	{
		if (mOrderDirty)
			RebuildOrder();
		for (int32_t slot : mOrder)
		{
			Mat44 world = LocalMatrix(slot);
			if (int32_t parent = mParent[slot]; parent != 0)
				world = LoadWorld(parent) * world;
			world.StoreFloat4x4(reinterpret_cast<Float4 *>(&mWorld[slot * 16]));
		}
	}

	void TransformStore::SetWorldPose(int32_t inSlot, Vec3Arg inPosition, QuatArg inRotation)
	{
		Vec3 position = inPosition;
		Quat rotation = inRotation;
		if (int32_t parent = mParent[inSlot]; parent != 0)
		{
			Mat44 parent_world = LoadWorld(parent);
			Vec3 parent_scale;
			position = parent_world.Inversed() * inPosition;
			rotation = (parent_world.Decompose(parent_scale).GetQuaternion().Conjugated() * inRotation).Normalized();
		}
		position.StoreFloat3(reinterpret_cast<Float3 *>(&mPosition[inSlot * 3]));
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

SN_API int32_t sn_transforms_columns(int32_t transforms, sn_transform_columns *out_columns)
{
	TransformStore *store = GetStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (out_columns == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "out_columns is required");
	*out_columns = { store->mPosition.data(), store->mRotation.data(), store->mScale.data(), store->mParent.data(), store->mWorld.data(), store->Capacity() };
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

SN_API int32_t sn_transforms_slot_limit(int32_t transforms)
{
	TransformStore *store = GetStore(transforms);
	return store != nullptr ? int32_t(store->SlotLimit()) : SN_ERROR_INVALID_HANDLE;
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
