// stride-native 运行时模块（stride-native-core）的 C API：变换、网格解码、动画。物理、贴图转码、导航是单独的侧模块，
// 见 Physics.h、Texture.h、Navigation.h；第三方插件用 Plugin.h。
// Handles are positive int32 values; negative return values are SN_ERROR_* codes.
#pragma once

#include <StrideNative/Core.h>

// ---------------------------------------------------------------------------------------------
// 变换：实体的本地位姿和世界矩阵，按结构数组放在模块内存里
// ---------------------------------------------------------------------------------------------

/// 一个变换存储的各列。每个实体占一个槽位，槽位从 1 开始：0 是空槽位，所以 parent 为 0 表示根，
/// 结构体清零就是"没有变换"。指针在存储销毁前不变，JS 保留视图直接读写。
typedef struct sn_transform_columns
{
	float *position;				///< 本地位置，每个槽位 xyz
	float *rotation;				///< 本地旋转，每个槽位一个单位四元数 xyzw
	float *scale;					///< 本地缩放，每个槽位 xyz
	const int32_t *parent;			///< 父槽位，0 表示根；只能用 sn_transforms_set_parent 改
	const float *world;				///< 世界矩阵，每个槽位 16 个 float，列主序（平移在 12..14），由 sn_transforms_update 写
	uint32_t capacity;				///< 最大槽位号；各列都有 capacity + 1 项（含槽位 0）
} sn_transform_columns;

/// 创建能放 capacity 个槽位的变换存储，返回句柄。
SN_API int32_t sn_transforms_create(uint32_t capacity);
SN_API void sn_transforms_destroy(int32_t transforms);
SN_API int32_t sn_transforms_columns(int32_t transforms, sn_transform_columns *out_columns);
/// 分配 count 个槽位写进 out_slots：位置 0、旋转单位、缩放 1、没有父节点。返回 count；容量不够时什么都不分配。
SN_API int32_t sn_transforms_alloc(int32_t transforms, int32_t *out_slots, uint32_t count);
/// 释放这些槽位。还有子节点留着的槽位不能释放（一起释放的子节点不算），这时什么都不释放。
SN_API int32_t sn_transforms_free(int32_t transforms, const int32_t *slots, uint32_t count);
/// 把 slot 挂到 parent 下（parent 为 0 表示变成根）。不能形成环。
SN_API int32_t sn_transforms_set_parent(int32_t transforms, int32_t slot, int32_t parent);
/// 按父节点在前的顺序算出所有槽位的世界矩阵：world = parent.world * T * R * S。
SN_API int32_t sn_transforms_update(int32_t transforms);
/// 发出去过的最大槽位号（释放了也不降）：用到的槽位都在 1..它 之内，上传世界矩阵时只传这一段。
SN_API int32_t sn_transforms_slot_limit(int32_t transforms);

// ---------------------------------------------------------------------------------------------
// Mesh decoding (EXT_meshopt_compression / KHR_meshopt_compression)
// ---------------------------------------------------------------------------------------------

enum
{
	SN_MESHOPT_MODE_ATTRIBUTES = 0,
	SN_MESHOPT_MODE_TRIANGLES = 1,
	SN_MESHOPT_MODE_INDICES = 2,
};

enum
{
	SN_MESHOPT_FILTER_NONE = 0,
	SN_MESHOPT_FILTER_OCTAHEDRAL = 1,
	SN_MESHOPT_FILTER_QUATERNION = 2,
	SN_MESHOPT_FILTER_EXPONENTIAL = 3,
	SN_MESHOPT_FILTER_COLOR = 4,
};

/// Decodes one glTF buffer view into dst (count * stride bytes).
SN_API int32_t sn_meshopt_decode(void *dst, uint32_t count, uint32_t stride, const void *src, uint32_t src_size, int32_t mode, int32_t filter);

typedef struct sn_meshopt_request
{
	void *dst;
	uint32_t count;
	uint32_t stride;
	const void *src;
	uint32_t src_size;
	int32_t mode;
	int32_t filter;
	int32_t result;			///< Written by the task: SN_OK or an error code
} sn_meshopt_request;

/// Decodes many buffer views in parallel. The requests and buffers must stay alive until the task
/// finishes. Returns a task; it fails if any request failed (see each request's result).
SN_API int32_t sn_meshopt_decode_batch(sn_meshopt_request *requests, uint32_t count);

// ---------------------------------------------------------------------------------------------
// Animation (ozz-animation runtime)
// ---------------------------------------------------------------------------------------------

/// Loads ozz archives (as produced by stride-tools). Return handles.
SN_API int32_t sn_anim_load_skeleton(const void *data, uint32_t size);
SN_API int32_t sn_anim_load_animation(const void *data, uint32_t size);
SN_API void sn_anim_destroy_skeleton(int32_t skeleton);
SN_API void sn_anim_destroy_animation(int32_t animation);
SN_API int32_t sn_anim_skeleton_joint_count(int32_t skeleton);
SN_API float sn_anim_animation_duration(int32_t animation);

/// An instance owns the sampling context and the model-space matrices of one character.
SN_API int32_t sn_anim_instance_create(int32_t skeleton);
SN_API void sn_anim_instance_destroy(int32_t instance);
/// joint_count column-major 4x4 float matrices, valid while the instance lives.
SN_API const float *sn_anim_instance_model_matrices(int32_t instance);

typedef struct sn_anim_sample_request
{
	int32_t instance;
	int32_t animation;
	float ratio;					///< Normalized time in [0, 1]
	int32_t result;					///< Written by the call
} sn_anim_sample_request;

/// Samples and converts to model space for every request, in parallel; returns when all are done.
SN_API int32_t sn_anim_sample_batch(sn_anim_sample_request *requests, uint32_t count);
