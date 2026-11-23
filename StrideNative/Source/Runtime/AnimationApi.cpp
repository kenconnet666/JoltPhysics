// 骨骼动画（ozz-animation）：每个实例按一段后缀程序算姿势（采样剪辑、按 Stride 的 AnimationBlender 混合），
// 换到模型空间，再按蒙皮算出蒙皮矩阵；一批角色在任务系统上并行。
#include <Core/Core.h>
#include <Core/Debug.h>
#include <Core/HandleTable.h>
#include <StrideNative/Runtime.h>

#include <ozz/animation/runtime/animation.h>
#include <ozz/animation/runtime/ik_aim_job.h>
#include <ozz/animation/runtime/ik_two_bone_job.h>
#include <ozz/animation/runtime/local_to_model_job.h>
#include <ozz/animation/runtime/sampling_job.h>
#include <ozz/animation/runtime/skeleton.h>
#include <ozz/base/containers/vector.h>
#include <ozz/base/io/archive.h>
#include <ozz/base/io/stream.h>
#include <ozz/base/maths/simd_math.h>
#include <ozz/base/maths/simd_quaternion.h>
#include <ozz/base/maths/soa_transform.h>
#include <ozz/base/span.h>

#include <algorithm>
#include <array>
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

	struct Joint
	{
		float t[3];
		float r[4];		///< xyzw
		float s[3];
	};

	/// 重定向：源剪辑在源骨架上采样，按关节对照搬到目标骨架（sn_anim_retarget）
	struct Retarget
	{
		int32_t								mSourceAnimation = 0;	///< 句柄：每次用时查（源剪辑先销毁了就报错）
		int32_t								mSourceSkeleton = 0;
		int32_t								mTargetSkeleton = 0;
		std::vector<int32_t>				mSourceOf;				///< 每个目标关节对应的源关节，-1 是没有
		std::vector<std::array<float, 4>>	mCorrection;			///< 每个目标关节：源静止的模型旋转的逆 × 目标静止的模型旋转
		std::vector<Joint>					mTargetRest;			///< 目标骨架的静止局部姿势（逐关节）
		int32_t								mTargetHips = -1, mSourceHips = -1;
		float								mHipsScale = 1.0f;
		float								mSourceHipsRest[3] = { 0, 0, 0 };
		float								mTargetHipsRest[3] = { 0, 0, 0 };
	};

	struct AnimationAsset
	{
		Animation							mAnimation;
		std::unique_ptr<Retarget>			mRetarget;				///< 重定向出来的句柄才有；那时 mAnimation 是空的
	};

	/// 按关节的权重（个数补到 SoA 的整数倍，补的是 0）
	struct Mask
	{
		std::vector<float>					mWeights;
	};

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
		std::vector<sn_anim_ik>					mIks;		///< 每次求值以后按顺序做
		ozz::vector<SoaTransform>				mSnapshot;	///< sn_anim_instance_snapshot 存的局部姿势；空的是没存过
		ozz::vector<SoaTransform>				mSourcePose;	///< 重定向：源剪辑在源骨架上的局部姿势
		ozz::vector<ozz::math::Float4x4>		mSourceModels;	///< 重定向：源骨架的模型空间矩阵

		ozz::animation::SamplingJob::Context &ContextFor(const Animation &inAnimation)
		{
			for (auto &[animation, context] : mContexts)
				if (animation == &inAnimation)
					return *context;
			if (mContexts.size() >= cMaxContexts)
				mContexts.erase(mContexts.begin());
			auto context = std::make_unique<ozz::animation::SamplingJob::Context>();
			context->Resize(inAnimation.num_tracks());
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

	/// inWeights 不为空时每个关节的系数再乘它（遮罩）
	void BlendPoses(int32_t inType, const ozz::vector<SoaTransform> &inA, const ozz::vector<SoaTransform> &inB, float inFactor, ozz::vector<SoaTransform> &outPose, const float *inWeights = nullptr)
	{
		Joint a[4], b[4], result[4];
		for (size_t i = 0; i < inA.size(); ++i)
		{
			Unpack(inA[i], a);
			Unpack(inB[i], b);
			for (int j = 0; j < 4; ++j)
				BlendJoint(inType, a[j], b[j], inWeights != nullptr ? inFactor * inWeights[i * 4 + size_t(j)] : inFactor, result[j]);
			Pack(result, outPose[i]);
		}
	}

	// ---------------------------------------------------------------------------------------------

	void Conjugate(const float *inQ, float *outQ)
	{
		outQ[0] = -inQ[0];
		outQ[1] = -inQ[1];
		outQ[2] = -inQ[2];
		outQ[3] = inQ[3];
	}

	/// 四元数转向量：v + 2w(q × v) + 2 q × (q × v)
	void Rotate(const float *inQ, const float *inV, float *outV)
	{
		float t[3] = { 2 * (inQ[1] * inV[2] - inQ[2] * inV[1]), 2 * (inQ[2] * inV[0] - inQ[0] * inV[2]), 2 * (inQ[0] * inV[1] - inQ[1] * inV[0]) };
		float c[3] = { inQ[1] * t[2] - inQ[2] * t[1], inQ[2] * t[0] - inQ[0] * t[2], inQ[0] * t[1] - inQ[1] * t[0] };
		for (int k = 0; k < 3; ++k)
			outV[k] = inV[k] + inQ[3] * t[k] + c[k];
	}

	/// 模型矩阵的旋转和平移（ToAffine）
	void Decompose(const ozz::math::Float4x4 &inMatrix, float *outRotation, float *outTranslation)
	{
		ozz::math::SimdFloat4 translation, rotation, scale;
		ozz::math::ToAffine(inMatrix, &translation, &rotation, &scale);
		alignas(16) float t[4], r[4];
		ozz::math::StorePtr(translation, t);
		ozz::math::StorePtr(rotation, r);
		std::copy(r, r + 4, outRotation);
		if (outTranslation != nullptr)
			std::copy(t, t + 3, outTranslation);
	}

	/// 骨架的静止姿势的模型空间矩阵
	ozz::vector<ozz::math::Float4x4> RestModels(const Skeleton &inSkeleton)
	{
		ozz::vector<ozz::math::Float4x4> models(size_t(inSkeleton.num_joints()));
		ozz::animation::LocalToModelJob job;
		job.skeleton = &inSkeleton;
		job.input = inSkeleton.joint_rest_poses();
		job.output = ozz::make_span(models);
		job.Run();
		return models;
	}

	/// 重定向的剪辑在 inTime 秒的姿势写进 outPose（目标骨架的局部 SoA）。注意 Multiply(a, b) 按 Stride 的约定是 b·a
	int32_t SampleRetargeted(Instance &ioInstance, const Retarget &inRetarget, float inTime, ozz::vector<SoaTransform> &outPose)
	{
		AnimationAsset *source = GetTable<AnimationAsset>().Get(inRetarget.mSourceAnimation);
		SkeletonAsset *sourceSkeleton = GetTable<SkeletonAsset>().Get(inRetarget.mSourceSkeleton);
		if (source == nullptr || sourceSkeleton == nullptr || source->mRetarget != nullptr)
			return SN_ERROR_INVALID_HANDLE;
		const Skeleton &from = sourceSkeleton->mSkeleton;
		const Animation &animation = source->mAnimation;
		ioInstance.mSourcePose.resize(size_t(from.num_soa_joints()));
		ioInstance.mSourceModels.resize(size_t(from.num_joints()));
		ozz::animation::SamplingJob sampling;
		sampling.animation = &animation;
		sampling.context = &ioInstance.ContextFor(animation);
		float duration = animation.duration();
		sampling.ratio = duration > 0 ? std::clamp(inTime / duration, 0.0f, 1.0f) : 0.0f;
		sampling.output = ozz::make_span(ioInstance.mSourcePose);
		if (!sampling.Run())
			return SN_ERROR_FAILED;
		ozz::animation::LocalToModelJob to_model;
		to_model.skeleton = &from;
		to_model.input = ozz::make_span(ioInstance.mSourcePose);
		to_model.output = ozz::make_span(ioInstance.mSourceModels);
		if (!to_model.Run())
			return SN_ERROR_FAILED;

		// 目标关节按顺序（父在前）：模型空间的旋转 = 源关节现在的模型旋转 × 修正，没有对应的跟着父关节；局部 = 父的逆 × 自己
		const Skeleton &to = *ioInstance.mSkeleton;
		size_t count = size_t(to.num_joints());
		std::vector<std::array<float, 4>> model(count);
		std::vector<Joint> locals(inRetarget.mTargetRest);
		for (size_t j = 0; j < count; ++j)
		{
			int parent = to.joint_parents()[j];
			int s = inRetarget.mSourceOf[j];
			float parentRotation[4] = { 0, 0, 0, 1 };
			if (parent >= 0)
				std::copy(model[size_t(parent)].begin(), model[size_t(parent)].end(), parentRotation);
			if (s < 0)
			{
				Multiply(locals[j].r, parentRotation, model[j].data());
				continue;
			}
			float sourceRotation[4];
			Decompose(ioInstance.mSourceModels[size_t(s)], sourceRotation, nullptr);
			Multiply(inRetarget.mCorrection[j].data(), sourceRotation, model[j].data());
			Normalize(model[j].data());
			float inverse[4];
			Conjugate(parentRotation, inverse);
			Multiply(model[j].data(), inverse, locals[j].r);
			Normalize(locals[j].r);
		}
		// 髋：源的髋在模型空间里离静止位置挪了多少，目标按髋高的比例挪；换到父关节的空间
		if (inRetarget.mTargetHips >= 0 && inRetarget.mSourceHips >= 0)
		{
			float rotation[4], position[3];
			Decompose(ioInstance.mSourceModels[size_t(inRetarget.mSourceHips)], rotation, position);
			float target[3];
			for (int k = 0; k < 3; ++k)
				target[k] = inRetarget.mTargetHipsRest[k] + (position[k] - inRetarget.mSourceHipsRest[k]) * inRetarget.mHipsScale;
			// 父关节的模型空间：静止的平移链加上面算的旋转（髋的父一般是根节点，不动）
			int parent = to.joint_parents()[size_t(inRetarget.mTargetHips)];
			float parentPosition[3] = { 0, 0, 0 }, parentRotation[4] = { 0, 0, 0, 1 };
			std::vector<int> chain;
			for (int p = parent; p >= 0; p = to.joint_parents()[size_t(p)])
				chain.push_back(p);
			for (auto it = chain.rbegin(); it != chain.rend(); ++it)
			{
				float offset[3];
				Rotate(parentRotation, locals[size_t(*it)].t, offset);
				for (int k = 0; k < 3; ++k)
					parentPosition[k] += offset[k];
				float next[4];
				Multiply(locals[size_t(*it)].r, parentRotation, next);
				std::copy(next, next + 4, parentRotation);
			}
			float delta[3] = { target[0] - parentPosition[0], target[1] - parentPosition[1], target[2] - parentPosition[2] }, inverse[4];
			Conjugate(parentRotation, inverse);
			Rotate(inverse, delta, locals[size_t(inRetarget.mTargetHips)].t);
		}
		Joint lanes[4];
		for (size_t soa = 0; soa < outPose.size(); ++soa)
		{
			for (int lane = 0; lane < 4; ++lane)
			{
				size_t j = soa * 4 + size_t(lane);
				lanes[lane] = j < count ? locals[j] : Joint { { 0, 0, 0 }, { 0, 0, 0, 1 }, { 1, 1, 1 } };
			}
			Pack(lanes, outPose[soa]);
		}
		return SN_OK;
	}

	/// 局部（SoA）姿势里第 inJoint 个关节的旋转右乘 inCorrection（同 ozz 样例的 MultiplySoATransformQuaternion）
	void MultiplyLocalRotation(ozz::vector<SoaTransform> &ioLocals, int inJoint, const ozz::math::SimdQuaternion &inCorrection)
	{
		SoaTransform &soa = ioLocals[size_t(inJoint) / 4];
		ozz::math::SimdQuaternion quaternions[4];
		ozz::math::Transpose4x4(&soa.rotation.x, &quaternions->xyzw);
		quaternions[inJoint & 3] = quaternions[inJoint & 3] * inCorrection;
		ozz::math::Transpose4x4(&quaternions->xyzw, &soa.rotation.x);
	}

	ozz::math::SimdFloat4 Load3(const float *inValues, float inW)
	{
		return ozz::math::simd_float4::Load(inValues[0], inValues[1], inValues[2], inW);
	}

	/// 按顺序做实例的 IK：局部姿势在 mStack[0]、模型空间的在 mModels，改完从链的起点往下重算
	bool ApplyIk(Instance &ioInstance)
	{
		const Skeleton &skeleton = *ioInstance.mSkeleton;
		ozz::vector<SoaTransform> &locals = ioInstance.mStack[0];
		for (const sn_anim_ik &ik : ioInstance.mIks)
		{
			if (ik.weight <= 0.0f)
				continue;
			int from = ik.joints[0];
			// 相对的目标：加在这一步开始时末端关节（看向：那个关节）的位置上
			ozz::math::SimdFloat4 target = Load3(ik.target, 1.0f);
			if ((ik.flags & SN_ANIM_IK_RELATIVE) != 0)
			{
				int end = ik.type == SN_ANIM_IK_TWO_BONE ? ik.joints[2] : ik.joints[0];
				target = ozz::math::SetW(ioInstance.mModels[size_t(end)].cols[3] + Load3(ik.target, 0.0f), ozz::math::simd_float4::one());
			}
			if (ik.type == SN_ANIM_IK_OFFSET)
			{
				// 模型空间的平移换到父关节空间：父关节的逆矩阵作用到方向上（根关节的父空间就是模型空间）
				int parent = skeleton.joint_parents()[ik.joints[0]];
				ozz::math::SimdFloat4 offset = Load3(ik.target, 0.0f) * ozz::math::simd_float4::Load1(std::min(ik.weight, 1.0f));
				if (parent != ozz::animation::Skeleton::kNoParent)
					offset = ozz::math::TransformVector(ozz::math::Invert(ioInstance.mModels[size_t(parent)]), offset);
				SoaTransform &soa = locals[size_t(ik.joints[0]) / 4];
				ozz::math::SimdFloat4 translations[4];
				ozz::math::Transpose3x4(&soa.translation.x, translations);
				translations[ik.joints[0] & 3] = translations[ik.joints[0] & 3] + offset;
				ozz::math::Transpose4x3(translations, &soa.translation.x);
			}
			else if (ik.type == SN_ANIM_IK_TWO_BONE)
			{
				ozz::animation::IKTwoBoneJob job;
				job.target = target;
				job.pole_vector = Load3(ik.pole, 0.0f);
				job.mid_axis = Load3(ik.axis, 0.0f);
				job.weight = std::min(ik.weight, 1.0f);
				job.soften = ik.soften;
				job.twist_angle = ik.twist;
				job.start_joint = &ioInstance.mModels[size_t(ik.joints[0])];
				job.mid_joint = &ioInstance.mModels[size_t(ik.joints[1])];
				job.end_joint = &ioInstance.mModels[size_t(ik.joints[2])];
				ozz::math::SimdQuaternion start_correction, mid_correction;
				job.start_joint_correction = &start_correction;
				job.mid_joint_correction = &mid_correction;
				if (!job.Run())
					return false;
				MultiplyLocalRotation(locals, ik.joints[0], start_correction);
				MultiplyLocalRotation(locals, ik.joints[1], mid_correction);
			}
			else
			{
				ozz::animation::IKAimJob job;
				job.target = target;
				job.forward = Load3(ik.axis, 0.0f);
				job.up = Load3(ik.up, 0.0f);
				job.offset = Load3(ik.offset, 1.0f);
				job.pole_vector = Load3(ik.pole, 0.0f);
				job.weight = std::min(ik.weight, 1.0f);
				job.twist_angle = ik.twist;
				job.joint = &ioInstance.mModels[size_t(ik.joints[0])];
				ozz::math::SimdQuaternion correction;
				job.joint_correction = &correction;
				if (!job.Run())
					return false;
				MultiplyLocalRotation(locals, ik.joints[0], correction);
			}
			ozz::animation::LocalToModelJob to_model;
			to_model.skeleton = &skeleton;
			to_model.input = ozz::make_span(locals);
			to_model.output = ozz::make_span(ioInstance.mModels);
			to_model.from = from;
			if (!to_model.Run())
				return false;
		}
		return true;
	}

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
				if (asset->mRetarget != nullptr)
				{
					SkeletonAsset *target = GetTable<SkeletonAsset>().Get(asset->mRetarget->mTargetSkeleton);
					if (depth >= cMaxStack || target == nullptr || &target->mSkeleton != &skeleton)
						return SN_ERROR_INVALID_ARGUMENT;
					int32_t result = SampleRetargeted(*instance, *asset->mRetarget, op.time, instance->mStack[depth]);
					if (result != SN_OK)
						return result;
					++depth;
					continue;
				}
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
			else if (op.type == SN_ANIM_PUSH_SNAPSHOT)
			{
				if (depth >= cMaxStack || instance->mSnapshot.size() != instance->mStack[depth].size())
					return SN_ERROR_INVALID_ARGUMENT;
				std::copy(instance->mSnapshot.begin(), instance->mSnapshot.end(), instance->mStack[depth].begin());
				++depth;
			}
			else if (op.type == SN_ANIM_BLEND_MASKED || op.type == SN_ANIM_ADD_MASKED)
			{
				Mask *mask = GetTable<Mask>().Get(op.animation);
				if (mask == nullptr)
					return SN_ERROR_INVALID_HANDLE;
				if (depth < 2 || mask->mWeights.size() < instance->mStack[0].size() * 4)
					return SN_ERROR_INVALID_ARGUMENT;
				BlendPoses(op.type == SN_ANIM_BLEND_MASKED ? SN_ANIM_BLEND : SN_ANIM_ADD, instance->mStack[depth - 2], instance->mStack[depth - 1], op.factor, instance->mStack[depth - 2], mask->mWeights.data());
				--depth;
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
		if (!to_model.Run())
			return SN_ERROR_FAILED;
		if (instance->mIks.empty())
			return SN_OK;
		// IK 改局部旋转：静止姿势时先拷一份到栈底
		if (depth == 0)
		{
			std::copy(skeleton.joint_rest_poses().begin(), skeleton.joint_rest_poses().end(), instance->mStack[0].begin());
		}
		return ApplyIk(*instance) ? SN_OK : SN_ERROR_FAILED;
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
	if (asset != nullptr && asset->mRetarget != nullptr)
	{
		AnimationAsset *source = GetTable<AnimationAsset>().Get(asset->mRetarget->mSourceAnimation);
		return source != nullptr ? source->mAnimation.duration() : -1.0f;
	}
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

SN_API int32_t sn_anim_instance_snapshot(int32_t instance, const float *root_correction)
{
	Instance *i = GetTable<Instance>().Get(instance);
	if (i == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	const Skeleton &skeleton = *i->mSkeleton;
	int joints = skeleton.num_joints();
	ozz::math::Float4x4 correction = ozz::math::Float4x4::identity();
	if (root_correction != nullptr)
		for (int c = 0; c < 4; ++c)
			correction.cols[c] = ozz::math::simd_float4::LoadPtrU(root_correction + c * 4);
	i->mSnapshot.resize(size_t(skeleton.num_soa_joints()));
	// 模型空间换回局部：根关节 = 修正 × 模型矩阵，别的 = 父关节的逆 × 自己；用不到的 SoA 通道填单位变换
	Joint lanes[4];
	for (int soa = 0; soa < skeleton.num_soa_joints(); ++soa)
	{
		for (int lane = 0; lane < 4; ++lane)
		{
			Joint &joint = lanes[lane];
			joint = { { 0, 0, 0 }, { 0, 0, 0, 1 }, { 1, 1, 1 } };
			int j = soa * 4 + lane;
			if (j >= joints)
				continue;
			int parent = skeleton.joint_parents()[j];
			ozz::math::Float4x4 local = parent == ozz::animation::Skeleton::kNoParent ? correction * i->mModels[size_t(j)]
				: ozz::math::Invert(i->mModels[size_t(parent)]) * i->mModels[size_t(j)];
			ozz::math::SimdFloat4 translation, rotation, scale;
			if (!ozz::math::ToAffine(local, &translation, &rotation, &scale))
				return SetLastError(SN_ERROR_FAILED, "sn_anim_instance_snapshot: joint " + std::to_string(j) + " has a degenerate matrix");
			alignas(16) float t[4], r[4], s[4];
			ozz::math::StorePtr(translation, t);
			ozz::math::StorePtr(rotation, r);
			ozz::math::StorePtr(scale, s);
			std::copy(t, t + 3, joint.t);
			std::copy(r, r + 4, joint.r);
			std::copy(s, s + 3, joint.s);
		}
		Pack(lanes, i->mSnapshot[size_t(soa)]);
	}
	return SN_OK;
}

SN_API int32_t sn_anim_instance_set_ik(int32_t instance, const sn_anim_ik *iks, uint32_t count)
{
	Instance *i = GetTable<Instance>().Get(instance);
	if (i == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (count > 0 && iks == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_anim_instance_set_ik: iks are required");
	int joints = i->mSkeleton->num_joints();
	for (uint32_t k = 0; k < count; ++k)
	{
		const sn_anim_ik &ik = iks[k];
		if (ik.type != SN_ANIM_IK_TWO_BONE && ik.type != SN_ANIM_IK_AIM && ik.type != SN_ANIM_IK_OFFSET)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "ik " + std::to_string(k) + ": unknown type " + std::to_string(ik.type));
		int used = ik.type == SN_ANIM_IK_TWO_BONE ? 3 : 1;
		for (int j = 0; j < used; ++j)
			if (ik.joints[j] < 0 || ik.joints[j] >= joints)
				return SetLastError(SN_ERROR_INVALID_ARGUMENT, "ik " + std::to_string(k) + ": joint " + std::to_string(ik.joints[j]) + " is out of range (the skeleton has " + std::to_string(joints) + ")");
	}
	i->mIks.assign(iks, iks + count);
	return int32_t(count);
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

SN_API int32_t sn_anim_retarget(int32_t source_animation, int32_t source_skeleton, int32_t target_skeleton, const int32_t *source_of_target,
	int32_t target_hips, int32_t source_hips)
{
	AnimationAsset *source = Lookup<AnimationAsset>(source_animation, "animation");
	SkeletonAsset *from = Lookup<SkeletonAsset>(source_skeleton, "skeleton");
	SkeletonAsset *to = Lookup<SkeletonAsset>(target_skeleton, "skeleton");
	if (source == nullptr || from == nullptr || to == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (source->mRetarget != nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_anim_retarget: the source is itself retargeted");
	if (source->mAnimation.num_tracks() != from->mSkeleton.num_joints())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_anim_retarget: the source animation is not for the source skeleton");
	if (source_of_target == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_anim_retarget: source_of_target is required");
	int sources = from->mSkeleton.num_joints(), targets = to->mSkeleton.num_joints();
	for (int j = 0; j < targets; ++j)
		if (source_of_target[j] < -1 || source_of_target[j] >= sources)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_anim_retarget: target joint " + std::to_string(j) + " maps to source joint " + std::to_string(source_of_target[j]) + " (the source has " + std::to_string(sources) + ")");
	if (target_hips >= targets || source_hips >= sources || (target_hips < 0) != (source_hips < 0))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_anim_retarget: hips must be both given (in range) or both -1");

	auto retarget = std::make_unique<Retarget>();
	retarget->mSourceAnimation = source_animation;
	retarget->mSourceSkeleton = source_skeleton;
	retarget->mTargetSkeleton = target_skeleton;
	retarget->mSourceOf.assign(source_of_target, source_of_target + targets);
	ozz::vector<ozz::math::Float4x4> sourceRest = RestModels(from->mSkeleton), targetRest = RestModels(to->mSkeleton);
	retarget->mCorrection.resize(size_t(targets));
	for (int j = 0; j < targets; ++j)
	{
		float targetRotation[4], sourceRotation[4] = { 0, 0, 0, 1 }, inverse[4];
		Decompose(targetRest[size_t(j)], targetRotation, nullptr);
		if (source_of_target[j] >= 0)
			Decompose(sourceRest[size_t(source_of_target[j])], sourceRotation, nullptr);
		Conjugate(sourceRotation, inverse);
		Multiply(targetRotation, inverse, retarget->mCorrection[size_t(j)].data());
	}
	// 目标的静止局部姿势逐关节拆出来
	retarget->mTargetRest.resize(size_t(targets));
	Joint lanes[4];
	for (int soa = 0; soa < to->mSkeleton.num_soa_joints(); ++soa)
	{
		Unpack(to->mSkeleton.joint_rest_poses()[size_t(soa)], lanes);
		for (int lane = 0; lane < 4 && soa * 4 + lane < targets; ++lane)
			retarget->mTargetRest[size_t(soa * 4 + lane)] = lanes[lane];
	}
	if (target_hips >= 0)
	{
		float rotation[4];
		Decompose(sourceRest[size_t(source_hips)], rotation, retarget->mSourceHipsRest);
		Decompose(targetRest[size_t(target_hips)], rotation, retarget->mTargetHipsRest);
		retarget->mTargetHips = target_hips;
		retarget->mSourceHips = source_hips;
		retarget->mHipsScale = std::abs(retarget->mSourceHipsRest[1]) > 1e-6f ? retarget->mTargetHipsRest[1] / retarget->mSourceHipsRest[1] : 1.0f;
	}
	auto asset = std::make_unique<AnimationAsset>();
	asset->mRetarget = std::move(retarget);
	int32_t handle = GetTable<AnimationAsset>().Add(std::move(asset));
	return handle;
}

SN_API int32_t sn_anim_mask_create(const float *weights, uint32_t joint_count)
{
	if (weights == nullptr || joint_count == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "a mask needs one weight per joint");
	auto mask = std::make_unique<Mask>();
	mask->mWeights.assign((joint_count + 3) / 4 * 4, 0.0f);
	for (uint32_t j = 0; j < joint_count; ++j)
		mask->mWeights[j] = std::clamp(weights[j], 0.0f, 1.0f);
	return GetTable<Mask>().Add(std::move(mask));
}

SN_API void sn_anim_mask_destroy(int32_t mask)
{
	GetTable<Mask>().Remove(mask);
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
