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
/// 链接（Stride 的 TransformLink，比如 ModelNodeLinkComponent 挂到骨骼节点）：slot 的世界矩阵改成
/// link_slot.world * matrix * T * R * S，不再看父节点（父子关系不变）。matrix 是列主序 4x4，拷进存储；每帧要变时再调一次。
/// link_slot 为 0 时去掉链接。link_slot 不能依赖 slot（会成环）。槽位释放时，它上面的和指向它的链接一起去掉。
SN_API int32_t sn_transforms_set_link(int32_t transforms, int32_t slot, int32_t link_slot, const float *matrix);
/// 按父节点（有链接时是链接的槽位）在前的顺序算出所有槽位的世界矩阵：world = parent.world * T * R * S。
SN_API int32_t sn_transforms_update(int32_t transforms);
/// 上一次 sn_transforms_update 时世界矩阵变了的槽位（和再上一次比；那之间分配、释放、sn_transforms_update_slot 改过的也算，
/// 每个槽位只出现一次）。out_slots 指向存储里的数组，下一次 sn_transforms_update 之前有效；返回个数。渲染、天空遮挡这些
/// 按它决定传哪些、要不要重画（没变就不做）。
SN_API int32_t sn_transforms_changed(int32_t transforms, const int32_t **out_slots);
/// 只算 slot 和它的祖先的世界矩阵（Stride 的 TransformComponent.UpdateWorldMatrix）：脚本在这一帧的 sn_transforms_update
/// 之前要用世界位姿时调用。
SN_API int32_t sn_transforms_update_slot(int32_t transforms, int32_t slot);
/// 发出去过的最大槽位号（释放了也不降）：用到的槽位都在 1..它 之内，上传世界矩阵时只传这一段。
SN_API int32_t sn_transforms_slot_limit(int32_t transforms);

/// 剔除用的一个视图（同 Stride 的 VisibilityGroup.TryCollect）。
typedef struct sn_cull_view
{
	/// 朝内的平面，xyz 法线、w：dot(n, p) + w > 0 在内侧（同 Stride 的 BoundingFrustum）。近、远平面放在最后两个。
	float planes[6][4];
	/// 6；阴影级联用 4，忽略近、远平面（Stride 的 VisiblityIgnoreDepthPlanes）
	uint32_t plane_count;
	/// 非 0 时量出可见物体的包围盒到 distance_plane 的最近、最远距离，写进 min_distance、max_distance
	int32_t measure_distance;
	/// xyz 法线、w：距离 = dot(n, p) - w（同 Stride 的 VisibilityGroup 里的相机近平面）
	float distance_plane[4];
	float min_distance;				///< 输出；没有可见物体时是 +inf
	float max_distance;				///< 输出；没有可见物体时是 -inf
} sn_cull_view;

/// 视锥剔除：实例的变换槽位按批次连续排在 slots 里，batch_ends[b] 是第 b 批的结束下标；同一批（同一个网格）共用一个本地
/// 包围盒，bounds 每批 6 个 float（中心 xyz、半长 xyz）。包围盒用槽位的世界矩阵变换（中心按点变换，半长按矩阵分量的绝对值
/// 变换，同 Stride 的 BoundingBoxExt.Transform），半长为 0 的包围盒总是可见、不参与量距离。第 v 个视图的结果写进
/// out_slots 的第 v * slot_count 段：每批可见的槽位按原顺序紧凑地放在这批原来的起点，个数写进 out_counts[v * batch_count + b]。
/// 视图数不设上限。
SN_API int32_t sn_transforms_cull(int32_t transforms, const int32_t *slots, uint32_t slot_count, const uint32_t *batch_ends, const float *bounds, uint32_t batch_count, sn_cull_view *views, uint32_t view_count, int32_t *out_slots, uint32_t *out_counts);

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

/// An instance owns the sampling contexts, the pose stack and the model-space matrices of one character.
SN_API int32_t sn_anim_instance_create(int32_t skeleton);
SN_API void sn_anim_instance_destroy(int32_t instance);
/// joint_count column-major 4x4 float matrices (joint-to-model), valid while the instance lives.
SN_API const float *sn_anim_instance_model_matrices(int32_t instance);

// A pose is computed by a small postfix program per instance, like Stride's AnimationOperation list:
// PUSH samples a clip onto the stack; BLEND, ADD and SUBTRACT pop the top two poses (a below b) and
// push the result. The blending math is Stride's AnimationBlender: per joint, translation and scale
// lerp(a, a op b) and rotation slerp(a, a op b) by factor, where for BLEND "a op b" is b, for ADD
// a + b (rotation b·a), for SUBTRACT a − b (rotation conj(b)·a).
enum
{
	SN_ANIM_PUSH = 0,
	SN_ANIM_BLEND = 1,
	SN_ANIM_ADD = 2,
	SN_ANIM_SUBTRACT = 3,
};

typedef struct sn_anim_op
{
	int32_t type;					///< SN_ANIM_*
	int32_t animation;				///< SN_ANIM_PUSH: the clip
	float time;						///< SN_ANIM_PUSH: seconds into the clip, clamped to [0, duration]
	float factor;					///< blends: 0 keeps a, 1 applies b fully
} sn_anim_op;

typedef struct sn_anim_evaluate_request
{
	int32_t instance;
	uint32_t first_op;				///< index into the ops array
	uint32_t op_count;				///< 0 evaluates the rest pose
	int32_t result;					///< Written by the call
} sn_anim_evaluate_request;

/// Runs every request's program (at most 8 poses deep, leaving one pose), converts to model space, in
/// parallel; returns when all are done.
SN_API int32_t sn_anim_evaluate_batch(const sn_anim_op *ops, sn_anim_evaluate_request *requests, uint32_t count);

/// A skin binds a mesh to joints of a skeleton: joints are skeleton joint indices, inverse_binds one
/// column-major 4x4 matrix per joint (mesh to joint space at bind time).
SN_API int32_t sn_anim_skin_create(const uint32_t *joints, const float *inverse_binds, uint32_t joint_count);
SN_API void sn_anim_skin_destroy(int32_t skin);
SN_API int32_t sn_anim_skin_joint_count(int32_t skin);

typedef struct sn_anim_skin_request
{
	int32_t instance;				///< evaluated with sn_anim_evaluate_batch
	int32_t skin;
	float *output;					///< joint_count column-major 4x4 matrices: model matrix of the joint times its inverse bind
	int32_t result;					///< Written by the call
} sn_anim_skin_request;

/// Writes the skinning matrices of every request, in parallel.
SN_API int32_t sn_anim_skin_batch(sn_anim_skin_request *requests, uint32_t count);
