// Skeletal animation sampling with ozz-animation; a batch of characters is sampled in parallel.
#include <Core/Core.h>
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

#include <memory>

namespace
{
	using namespace StrideNative;
	using ozz::animation::Animation;
	using ozz::animation::Skeleton;

	struct SkeletonAsset { Skeleton mSkeleton; };
	struct AnimationAsset { Animation mAnimation; };

	struct Instance
	{
		const Skeleton *							mSkeleton = nullptr;	///< Owned by the skeleton handle, which must outlive this
		ozz::animation::SamplingJob::Context		mContext;
		ozz::vector<ozz::math::SoaTransform>		mLocals;
		ozz::vector<ozz::math::Float4x4>			mModels;
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

	int32_t Sample(const sn_anim_sample_request &inRequest)
	{
		Instance *instance = GetTable<Instance>().Get(inRequest.instance);
		AnimationAsset *animation = GetTable<AnimationAsset>().Get(inRequest.animation);
		if (instance == nullptr || animation == nullptr)
			return SN_ERROR_INVALID_HANDLE;
		if (animation->mAnimation.num_tracks() != instance->mSkeleton->num_joints())
			return SN_ERROR_INVALID_ARGUMENT;

		ozz::animation::SamplingJob sampling;
		sampling.animation = &animation->mAnimation;
		sampling.context = &instance->mContext;
		sampling.ratio = inRequest.ratio;
		sampling.output = ozz::make_span(instance->mLocals);
		if (!sampling.Run())
			return SN_ERROR_FAILED;

		ozz::animation::LocalToModelJob to_model;
		to_model.skeleton = instance->mSkeleton;
		to_model.input = ozz::make_span(instance->mLocals);
		to_model.output = ozz::make_span(instance->mModels);
		return to_model.Run() ? SN_OK : SN_ERROR_FAILED;
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
	instance->mContext.Resize(asset->mSkeleton.num_joints());
	instance->mLocals.resize(asset->mSkeleton.num_soa_joints());
	instance->mModels.resize(asset->mSkeleton.num_joints());
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

SN_API int32_t sn_anim_sample_batch(sn_anim_sample_request *requests, uint32_t count)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (requests == nullptr && count > 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "requests is null");

	// Each request touches only its own instance, so requests for distinct instances run in parallel.
	ParallelFor(count, [requests](uint32_t inBegin, uint32_t inEnd)
	{
		for (uint32_t i = inBegin; i < inEnd; ++i)
			requests[i].result = Sample(requests[i]);
	});

	for (uint32_t i = 0; i < count; ++i)
		if (requests[i].result != SN_OK)
			return SetLastError(requests[i].result, "sampling request " + std::to_string(i) + " failed");
	return SN_OK;
}
