// Builds ozz skeleton and animation archives (loaded at runtime by sn_anim_load_*).
#include <Core/Core.h>
#include <StrideNative/Tools.h>

#include <ozz/animation/offline/animation_builder.h>
#include <ozz/animation/offline/raw_animation.h>
#include <ozz/animation/offline/raw_skeleton.h>
#include <ozz/animation/offline/skeleton_builder.h>
#include <ozz/animation/runtime/animation.h>
#include <ozz/animation/runtime/skeleton.h>
#include <ozz/base/io/archive.h>
#include <ozz/base/io/stream.h>

#include <cstring>
#include <vector>

namespace
{
	using namespace StrideNative;
	using ozz::animation::offline::RawAnimation;
	using ozz::animation::offline::RawSkeleton;

	template <class T>
	int32_t WriteArchive(const T &inObject, void **outData, uint32_t *outSize)
	{
		ozz::io::MemoryStream stream;
		ozz::io::OArchive archive(&stream);
		archive << inObject;

		uint32_t size = uint32_t(stream.Size());
		void *data = sn_alloc(size);
		stream.Seek(0, ozz::io::Stream::kSet);
		stream.Read(data, size);
		*outData = data;
		*outSize = size;
		return SN_OK;
	}
}

SN_API int32_t st_anim_build_skeleton(const st_joint *joints, uint32_t joint_count, const char *names, void **out_data, uint32_t *out_size)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (joints == nullptr || joint_count == 0 || out_data == nullptr || out_size == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "joints and outputs are required");

	// RawSkeleton is a tree of nested vectors; build it from the flat parent list, keeping each
	// joint's path of child indices because pointers into the vectors move as they grow.
	RawSkeleton raw;
	std::vector<std::vector<size_t>> paths(joint_count);
	const char *name = names;
	for (uint32_t i = 0; i < joint_count; ++i)
	{
		const st_joint &joint = joints[i];
		if (joint.parent >= int32_t(i))
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "joint " + std::to_string(i) + " must come after its parent");

		// Walk the parent's path from the roots down to the parent's children list.
		RawSkeleton::Joint::Children *siblings = &raw.roots;
		if (joint.parent >= 0)
		{
			paths[i] = paths[joint.parent];
			for (size_t step : paths[i])
				siblings = &(*siblings)[step].children;
		}

		RawSkeleton::Joint &added = siblings->emplace_back();
		paths[i].push_back(siblings->size() - 1);
		added.transform.translation = ozz::math::Float3(joint.translation[0], joint.translation[1], joint.translation[2]);
		added.transform.rotation = ozz::math::Quaternion(joint.rotation[0], joint.rotation[1], joint.rotation[2], joint.rotation[3]);
		added.transform.scale = ozz::math::Float3(joint.scale[0], joint.scale[1], joint.scale[2]);
		if (name != nullptr)
		{
			added.name = name;
			name += std::strlen(name) + 1;
		}
		else
			added.name = "joint" + std::to_string(i);
	}
	if (!raw.Validate())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "invalid skeleton");

	ozz::unique_ptr<ozz::animation::Skeleton> skeleton = ozz::animation::offline::SkeletonBuilder()(raw);
	if (!skeleton)
		return SetLastError(SN_ERROR_FAILED, "ozz SkeletonBuilder failed");
	return WriteArchive(*skeleton, out_data, out_size);
}

SN_API int32_t st_anim_build_animation(float duration, uint32_t joint_count, const uint32_t *key_counts, const float *keys, void **out_data, uint32_t *out_size)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (duration <= 0.0f || key_counts == nullptr || keys == nullptr || out_data == nullptr || out_size == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "duration, key counts, keys and outputs are required");

	RawAnimation raw;
	raw.duration = duration;
	raw.tracks.resize(joint_count);
	const float *key = keys;
	for (uint32_t j = 0; j < joint_count; ++j)
	{
		RawAnimation::JointTrack &track = raw.tracks[j];
		for (uint32_t k = 0; k < key_counts[j * 3 + 0]; ++k, key += 4)
			track.translations.push_back({ key[0], ozz::math::Float3(key[1], key[2], key[3]) });
		for (uint32_t k = 0; k < key_counts[j * 3 + 1]; ++k, key += 5)
			track.rotations.push_back({ key[0], ozz::math::Quaternion(key[1], key[2], key[3], key[4]) });
		for (uint32_t k = 0; k < key_counts[j * 3 + 2]; ++k, key += 4)
			track.scales.push_back({ key[0], ozz::math::Float3(key[1], key[2], key[3]) });
	}
	if (!raw.Validate())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "invalid animation (keys must be sorted and within the duration)");

	ozz::unique_ptr<ozz::animation::Animation> animation = ozz::animation::offline::AnimationBuilder()(raw);
	if (!animation)
		return SetLastError(SN_ERROR_FAILED, "ozz AnimationBuilder failed");
	return WriteArchive(*animation, out_data, out_size);
}
