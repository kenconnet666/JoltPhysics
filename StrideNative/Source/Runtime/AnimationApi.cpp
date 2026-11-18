// 骨骼动画（ozz-animation）：每个实例按一段后缀程序算姿势（采样剪辑、按 Stride 的 AnimationBlender 混合），
// 换到模型空间，再按蒙皮算出蒙皮矩阵；一批角色在任务系统上并行。
#include <Core/Core.h>
#include <Core/Debug.h>
#include <Core/HandleTable.h>
#include <StrideNative/Runtime.h>

#include <ozz/animation/runtime/animation.h>
#include <ozz/animation/runtime/local_to_model_job.h>
#include <ozz/animation/runtime/sampling_job.h>
#include <ozz/animation/runtime/skeleton.h>
#include <ozz/base/containers/vector.h>
#include <ozz/base/io/archive.h>
#include <ozz/base/io/stream.h>
#include <ozz/base/maths/simd_math.h>
#include <ozz/base/maths/soa_transform.h>
#include <ozz/base/span.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace
{
	using namespace StrideNative;
	using ozz::animation::Animation;
	using ozz::animation::Skeleton;
	using ozz::math::SoaTransform;

	/// 一个实例最多叠几层姿势（程序里同时在栈上的）。
	constexpr uint32_t cMaxStack = 8;
	/// 一个实例最多缓存几个剪辑的采样上下文（同一个剪辑连续采样时 ozz 能复用上一帧的关键帧位置）。
	constexpr size_t cMaxContexts = 16;

	struct SkeletonAsset { Skeleton mSkeleton; };
	struct AnimationAsset { Animation mAnimation; };

	struct Skin
	{
		std::vector<uint32_t>				mJoints;
		std::vector<ozz::math::Float4x4>	mInverseBinds;
	};

	struct Instance
	{
		const Skeleton *						mSkeleton = nullptr;	///< 属于骨架句柄，骨架要比实例活得久
		std::vector<std::pair<const Animation *, std::unique_ptr<ozz::animation::SamplingJob::Context>>> mContexts;
		std::vector<ozz::vector<SoaTransform>>	mStack;
		ozz::vector<ozz::math::Float4x4>		mModels;

		ozz::animation::SamplingJob::Context &ContextFor(const Animation &inAnimation)
		{
			for (auto &[animation, context] : mContexts)
				if (animation == &inAnimation)
					return *context;
			if (mContexts.size() >= cMaxContexts)
				mContexts.erase(mContexts.begin());
			auto context = std::make_unique<ozz::animation::SamplingJob::Context>();
			context->Resize(mSkeleton->num_joints());
			return *mContexts.emplace_back(&inAnimation, std::move(context)).second;
		}
	};

	template <class Asset, class Object>
	int32_t LoadArchive(const void *inData, uint32_t inSize, Object Asset::*inMember, const char *inWhat)
	{
		if (inData == nullptr || inSize == 0)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, std::string(inWhat) + " data is empty");

		ozz::io::MemoryStream stream;
		stream.Write(inData, inSize);
		stream.Seek(0, ozz::io::Stream::kSet);
		ozz::io::IArchive archive(&stream);
		if (!archive.TestTag<Object>())
			return SetLastError(SN_ERROR_FAILED, std::string("data is not an ozz ") + inWhat + " archive");

		auto asset = std::make_unique<Asset>();
		archive >> (*asset).*inMember;
		return GetTable<Asset>().Add(std::move(asset));
	}

	// ---------------------------------------------------------------------------------------------
	// Stride 的 AnimationBlender 的数学，逐关节标量计算（SoA 拆成 4 个关节）

	struct Joint
	{
		float t[3];
		float r[4];		///< xyzw
		float s[3];
	};

	void Unpack(const SoaTransform &inSoa, Joint outJoints[4])
	{
		alignas(16) float lanes[10][4];
		const ozz::math::SimdFloat4 *sources[10] = { &inSoa.translation.x, &inSoa.translation.y, &inSoa.translation.z,
			&inSoa.rotation.x, &inSoa.rotation.y, &inSoa.rotation.z, &inSoa.rotation.w, &inSoa.scale.x, &inSoa.scale.y, &inSoa.scale.z };
		for (int c = 0; c < 10; ++c)
			ozz::math::StorePtr(*sources[c], lanes[c]);
		for (int j = 0; j < 4; ++j)
		{
			for (int c = 0; c < 3; ++c)
				outJoints[j].t[c] = lanes[c][j];
			for (int c = 0; c < 4; ++c)
				outJoints[j].r[c] = lanes[3 + c][j];
			for (int c = 0; c < 3; ++c)
				outJoints[j].s[c] = lanes[7 + c][j];
		}
	}

	void Pack(const Joint inJoints[4], SoaTransform &outSoa)
	{
		alignas(16) float lanes[10][4];
		for (int j = 0; j < 4; ++j)
		{
			for (int c = 0; c < 3; ++c)
				lanes[c][j] = inJoints[j].t[c];
			for (int c = 0; c < 4; ++c)
				lanes[3 + c][j] = inJoints[j].r[c];
			for (int c = 0; c < 3; ++c)
				lanes[7 + c][j] = inJoints[j].s[c];
		}
		ozz::math::SimdFloat4 *targets[10] = { &outSoa.translation.x, &outSoa.translation.y, &outSoa.translation.z,
			&outSoa.rotation.x, &outSoa.rotation.y, &outSoa.rotation.z, &outSoa.rotation.w, &outSoa.scale.x, &outSoa.scale.y, &outSoa.scale.z };
		for (int c = 0; c < 10; ++c)
			*targets[c] = ozz::math::simd_float4::LoadPtr(lanes[c]);
	}

	/// Stride 的 Quaternion.Multiply(left, right)（右乘左：先转 left 再转 right）。
	void Multiply(const float *inLeft, const float *inRight, float *outResult)
	{
		float lx = inLeft[0], ly = inLeft[1], lz = inLeft[2], lw = inLeft[3];
		float rx = inRight[0], ry = inRight[1], rz = inRight[2], rw = inRight[3];
		outResult[0] = (rx * lw + lx * rw + ry * lz) - (rz * ly);
		outResult[1] = (ry * lw + ly * rw + rz * lx) - (rx * lz);
		outResult[2] = (rz * lw + lz * rw + rx * ly) - (ry * lx);
		outResult[3] = (rw * lw) - (rx * lx + ry * ly + rz * lz);
	}

	void Normalize(float *ioQ)
	{
		float length = std::sqrt(ioQ[0] * ioQ[0] + ioQ[1] * ioQ[1] + ioQ[2] * ioQ[2] + ioQ[3] * ioQ[3]);
		if (length > 0)
			for (int c = 0; c < 4; ++c)
				ioQ[c] /= length;
	}

	/// Stride 的 Quaternion.Slerp（含它对接近平行时的处理和符号）。
	void Slerp(const float *inStart, const float *inEnd, float inAmount, float *outResult)
	{
		constexpr float cZeroTolerance = 1e-6f;
		float dot = inStart[0] * inEnd[0] + inStart[1] * inEnd[1] + inStart[2] * inEnd[2] + inStart[3] * inEnd[3];
		float sign = dot > 0 ? 1.0f : dot < 0 ? -1.0f : 0.0f;
		float inverse, opposite;
		if (std::fabs(dot) > 1.0f - cZeroTolerance)
		{
			inverse = 1.0f - inAmount;
			opposite = inAmount * sign;
		}
		else
		{
			float angle = std::acos(std::fabs(dot));
			float inv_sin = 1.0f / std::sin(angle);
			inverse = std::sin((1.0f - inAmount) * angle) * inv_sin;
			opposite = std::sin(inAmount * angle) * inv_sin * sign;
		}
		for (int c = 0; c < 4; ++c)
			outResult[c] = inverse * inStart[c] + opposite * inEnd[c];
	}

	void BlendJoint(int32_t inType, const Joint &inA, const Joint &inB, float inFactor, Joint &outJoint)
	{
		// 目标：BLEND 是 b，ADD 是 a + b，SUBTRACT 是 a − b；再从 a 往目标插值 factor
		float t[3], s[3], r[4];
		for (int c = 0; c < 3; ++c)
		{
			t[c] = inType == SN_ANIM_BLEND ? inB.t[c] : inType == SN_ANIM_ADD ? inA.t[c] + inB.t[c] : inA.t[c] - inB.t[c];
			s[c] = inType == SN_ANIM_BLEND ? inB.s[c] : inType == SN_ANIM_ADD ? inA.s[c] + inB.s[c] : inA.s[c] - inB.s[c];
		}
		if (inType == SN_ANIM_BLEND)
			std::copy(inB.r, inB.r + 4, r);
		else if (inType == SN_ANIM_ADD)
		{
			Multiply(inA.r, inB.r, r);
			Normalize(r);
		}
		else
		{
			// Stride：Invert(b) 再 Multiply(Invert(b), a)；单位四元数的逆就是共轭除以长度平方
			float lengthSq = inB.r[0] * inB.r[0] + inB.r[1] * inB.r[1] + inB.r[2] * inB.r[2] + inB.r[3] * inB.r[3];
			float scale = lengthSq > 0 ? 1.0f / lengthSq : 0.0f;
			float inverse[4] = { -inB.r[0] * scale, -inB.r[1] * scale, -inB.r[2] * scale, inB.r[3] * scale };
			Multiply(inverse, inA.r, r);
			Normalize(r);
		}
		for (int c = 0; c < 3; ++c)
		{
			outJoint.t[c] = inA.t[c] + (t[c] - inA.t[c]) * inFactor;
			outJoint.s[c] = inA.s[c] + (s[c] - inA.s[c]) * inFactor;
		}
		Slerp(inA.r, r, inFactor, outJoint.r);
	}

	void BlendPoses(int32_t inType, const ozz::vector<SoaTransform> &inA, const ozz::vector<SoaTransform> &inB, float inFactor, ozz::vector<SoaTransform> &outPose)
	{
		Joint a[4], b[4], result[4];
		for (size_t i = 0; i < inA.size(); ++i)
		{
			Unpack(inA[i], a);
			Unpack(inB[i], b);
			for (int j = 0; j < 4; ++j)
				BlendJoint(inType, a[j], b[j], inFactor, result[j]);
			Pack(result, outPose[i]);
		}
	}

	// ---------------------------------------------------------------------------------------------

	int32_t Evaluate(const sn_anim_op *inOps, const sn_anim_evaluate_request &inRequest)
	{
		Instance *instance = GetTable<Instance>().Get(inRequest.instance);
		if (instance == nullptr)
			return SN_ERROR_INVALID_HANDLE;
		const Skeleton &skeleton = *instance->mSkeleton;

		uint32_t depth = 0;
		for (uint32_t i = 0; i < inRequest.op_count; ++i)
		{
			const sn_anim_op &op = inOps[inRequest.first_op + i];
			if (op.type == SN_ANIM_PUSH)
			{
				AnimationAsset *asset = GetTable<AnimationAsset>().Get(op.animation);
				if (asset == nullptr)
					return SN_ERROR_INVALID_HANDLE;
				const Animation &animation = asset->mAnimation;
				if (depth >= cMaxStack || animation.num_tracks() != skeleton.num_joints())
					return SN_ERROR_INVALID_ARGUMENT;
				ozz::animation::SamplingJob sampling;
				sampling.animation = &animation;
				sampling.context = &instance->ContextFor(animation);
				float duration = animation.duration();
				sampling.ratio = duration > 0 ? std::clamp(op.time / duration, 0.0f, 1.0f) : 0.0f;
				sampling.output = ozz::make_span(instance->mStack[depth]);
				if (!sampling.Run())
					return SN_ERROR_FAILED;
				++depth;
			}
			else if (op.type == SN_ANIM_BLEND || op.type == SN_ANIM_ADD || op.type == SN_ANIM_SUBTRACT)
			{
				if (depth < 2)
					return SN_ERROR_INVALID_ARGUMENT;
				// 结果写回 a 的位置：BlendPoses 逐个 SoA 块读完再写，原地可以
				BlendPoses(op.type, instance->mStack[depth - 2], instance->mStack[depth - 1], op.factor, instance->mStack[depth - 2]);
				--depth;
			}
			else
				return SN_ERROR_INVALID_ARGUMENT;
		}
		if (depth > 1)
			return SN_ERROR_INVALID_ARGUMENT;

		ozz::animation::LocalToModelJob to_model;
		to_model.skeleton = &skeleton;
		// 空程序：骨架的静止姿势
		if (depth == 0)
			to_model.input = skeleton.joint_rest_poses();
		else
			to_model.input = ozz::make_span(instance->mStack[0]);
		to_model.output = ozz::make_span(instance->mModels);
		return to_model.Run() ? SN_OK : SN_ERROR_FAILED;
	}

	int32_t SkinMatrices(const sn_anim_skin_request &inRequest)
	{
		Instance *instance = GetTable<Instance>().Get(inRequest.instance);
		Skin *skin = GetTable<Skin>().Get(inRequest.skin);
		if (instance == nullptr || skin == nullptr)
			return SN_ERROR_INVALID_HANDLE;
		if (inRequest.output == nullptr)
			return SN_ERROR_INVALID_ARGUMENT;
		for (size_t k = 0; k < skin->mJoints.size(); ++k)
		{
			uint32_t joint = skin->mJoints[k];
			if (joint >= instance->mModels.size())
				return SN_ERROR_INVALID_ARGUMENT;
			ozz::math::Float4x4 matrix = instance->mModels[joint] * skin->mInverseBinds[k];
			for (int c = 0; c < 4; ++c)
				ozz::math::StorePtrU(matrix.cols[c], inRequest.output + k * 16 + c * 4);
		}
		return SN_OK;
	}

	/// 批量请求：并行跑，任何一个失败时报出第一个失败的。
	template <class Request, class Work>
	int32_t RunBatch(Request *ioRequests, uint32_t inCount, const char *inWhat, Work inWork)
	{
		if (ioRequests == nullptr && inCount > 0)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "requests is null");
		ParallelFor(inCount, [&](uint32_t inBegin, uint32_t inEnd)
		{
			for (uint32_t i = inBegin; i < inEnd; ++i)
				ioRequests[i].result = inWork(ioRequests[i]);
		});
		for (uint32_t i = 0; i < inCount; ++i)
			if (ioRequests[i].result != SN_OK)
				return SetLastError(ioRequests[i].result, std::string(inWhat) + " request " + std::to_string(i) + " failed");
		return SN_OK;
	}
}

SN_API int32_t sn_anim_load_skeleton(const void *data, uint32_t size)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	return LoadArchive(data, size, &SkeletonAsset::mSkeleton, "skeleton");
}

SN_API int32_t sn_anim_load_animation(const void *data, uint32_t size)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	return LoadArchive(data, size, &AnimationAsset::mAnimation, "animation");
}

SN_API void sn_anim_destroy_skeleton(int32_t skeleton)
{
	GetTable<SkeletonAsset>().Remove(skeleton);
}

SN_API void sn_anim_destroy_animation(int32_t animation)
{
	GetTable<AnimationAsset>().Remove(animation);
}

SN_API int32_t sn_anim_skeleton_joint_count(int32_t skeleton)
{
	SkeletonAsset *asset = Lookup<SkeletonAsset>(skeleton, "skeleton");
	return asset != nullptr ? asset->mSkeleton.num_joints() : SN_ERROR_INVALID_HANDLE;
}

SN_API float sn_anim_animation_duration(int32_t animation)
{
	AnimationAsset *asset = Lookup<AnimationAsset>(animation, "animation");
	return asset != nullptr ? asset->mAnimation.duration() : -1.0f;
}

SN_API int32_t sn_anim_instance_create(int32_t skeleton)
{
	SkeletonAsset *asset = Lookup<SkeletonAsset>(skeleton, "skeleton");
	if (asset == nullptr)
		return SN_ERROR_INVALID_HANDLE;

	auto instance = std::make_unique<Instance>();
	instance->mSkeleton = &asset->mSkeleton;
	instance->mStack.resize(cMaxStack);
	for (ozz::vector<SoaTransform> &pose : instance->mStack)
		pose.resize(asset->mSkeleton.num_soa_joints());
	instance->mModels.resize(asset->mSkeleton.num_joints());
	// 求值之前是静止姿势
	ozz::animation::LocalToModelJob to_model;
	to_model.skeleton = &asset->mSkeleton;
	to_model.input = asset->mSkeleton.joint_rest_poses();
	to_model.output = ozz::make_span(instance->mModels);
	to_model.Run();
	return GetTable<Instance>().Add(std::move(instance));
}

SN_API void sn_anim_instance_destroy(int32_t instance)
{
	GetTable<Instance>().Remove(instance);
}

SN_API const float *sn_anim_instance_model_matrices(int32_t instance)
{
	Instance *i = Lookup<Instance>(instance, "animation instance");
	static_assert(sizeof(ozz::math::Float4x4) == 16 * sizeof(float), "Float4x4 must be 16 packed floats");
	return i != nullptr ? reinterpret_cast<const float *>(i->mModels.data()) : nullptr;
}

SN_API int32_t sn_anim_evaluate_batch(const sn_anim_op *ops, sn_anim_evaluate_request *requests, uint32_t count)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	SN_DEBUG_SCOPE("animation.evaluate");
	if (DebugEnabled())
		sn_debug_set_counter("animation.instances", count);
	return RunBatch(requests, count, "animation evaluation", [ops](const sn_anim_evaluate_request &inRequest) { return Evaluate(ops, inRequest); });
}

SN_API int32_t sn_anim_skin_create(const uint32_t *joints, const float *inverse_binds, uint32_t joint_count)
{
	if (joints == nullptr || inverse_binds == nullptr || joint_count == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "a skin needs joints and inverse binds");
	auto skin = std::make_unique<Skin>();
	skin->mJoints.assign(joints, joints + joint_count);
	skin->mInverseBinds.resize(joint_count);
	for (uint32_t k = 0; k < joint_count; ++k)
		for (int c = 0; c < 4; ++c)
			skin->mInverseBinds[k].cols[c] = ozz::math::simd_float4::LoadPtrU(inverse_binds + k * 16 + c * 4);
	return GetTable<Skin>().Add(std::move(skin));
}

SN_API void sn_anim_skin_destroy(int32_t skin)
{
	GetTable<Skin>().Remove(skin);
}

SN_API int32_t sn_anim_skin_joint_count(int32_t skin)
{
	Skin *s = Lookup<Skin>(skin, "skin");
	return s != nullptr ? int32_t(s->mJoints.size()) : SN_ERROR_INVALID_HANDLE;
}

SN_API int32_t sn_anim_skin_batch(sn_anim_skin_request *requests, uint32_t count)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	SN_DEBUG_SCOPE("animation.skin");
	return RunBatch(requests, count, "skinning", [](const sn_anim_skin_request &inRequest) { return SkinMatrices(inRequest); });
}
