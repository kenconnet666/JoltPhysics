// stride-native runtime C API: physics, decoding, navigation and animation.
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

// ---------------------------------------------------------------------------------------------
// Physics (smoke test only; the Stride-shaped physics API is a separate milestone)
// ---------------------------------------------------------------------------------------------

enum
{
	SN_BODY_DYNAMIC = 1,			///< sn_box flag; without it the box is static
};

typedef struct sn_box
{
	float position[3];				///< 没有绑定变换时的初始位置
	float half_extent[3];
	uint32_t flags;					///< SN_BODY_*
	/// 绑定的变换槽位，0 表示不绑定。绑定后初始位姿取这个槽位当前的世界位姿（缩放不影响形状），
	/// 动态物体每步结束把位姿写回槽位的本地位置和旋转（有父节点时换算到父节点空间）。
	int32_t transform;
} sn_box;

/// Creates a world that steps on the shared job system. Returns a world handle. transforms 是物体可以绑定的
/// 变换存储（0 表示不用）；它要比世界活得久，绑定的槽位在物体存在期间不能释放。
SN_API int32_t sn_physics_create(uint32_t max_bodies, int32_t transforms);
SN_API void sn_physics_destroy(int32_t world);
/// Adds count boxes as one batch (Jolt builds their broadphase nodes together). Returns the index of
/// the first new body; bodies are indexed in creation order.
SN_API int32_t sn_physics_add_boxes(int32_t world, const sn_box *boxes, uint32_t count);
/// Steps, then writes every body's position into the positions buffer.
SN_API int32_t sn_physics_step(int32_t world, float delta_time, int32_t collision_steps);
/// The world's positions buffer: xyz per body in creation order, room for max_bodies. It stays at this
/// address until the world is destroyed, so callers keep one view of it; add_boxes and step update it.
SN_API const float *sn_physics_positions(int32_t world);

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
// KTX2 / Basis Universal transcoding
// ---------------------------------------------------------------------------------------------

/// Transcode targets, named after the WebGPU format they upload to.
enum
{
	SN_TEXTURE_RGBA8 = 0,			///< rgba8unorm(-srgb)
	SN_TEXTURE_BC7 = 1,				///< bc7-rgba-unorm(-srgb), texture-compression-bc
	SN_TEXTURE_ASTC_4X4 = 2,		///< astc-4x4-unorm(-srgb), texture-compression-astc
	SN_TEXTURE_ETC2_RGBA8 = 3,		///< etc2-rgba8unorm(-srgb), texture-compression-etc2
};

typedef struct sn_ktx2_info
{
	uint32_t width;
	uint32_t height;
	uint32_t levels;
	uint32_t layers;				///< 0 for a plain 2D texture
	uint32_t faces;					///< 6 for cubemaps
	int32_t has_alpha;
	int32_t is_srgb;
	int32_t is_hdr;
} sn_ktx2_info;

SN_API int32_t sn_ktx2_get_info(const void *data, uint32_t size, sn_ktx2_info *out_info);

/// Transcodes every mip level in parallel. data must stay alive until the task finishes.
/// Outputs: one buffer per level (level 0 first) for layer 0 / face 0.
/// Results: [0] width, [1] height, [2] level count, [3] bytes per block (or per pixel for RGBA8).
SN_API int32_t sn_ktx2_transcode(const void *data, uint32_t size, int32_t target);

// ---------------------------------------------------------------------------------------------
// Navigation (Recast/Detour)
// ---------------------------------------------------------------------------------------------

typedef struct sn_nav_config
{
	float cell_size;
	float cell_height;
	float agent_height;
	float agent_radius;
	float agent_max_climb;
	float agent_max_slope_degrees;
	int32_t tile_size;				///< In cells; tiles are built in parallel
	float max_edge_length;
	float max_simplification_error;
	float min_region_size;
	float merge_region_size;
	float detail_sample_distance;
	float detail_sample_max_error;
} sn_nav_config;

/// Fills a config with the Recast demo defaults for a human-sized agent.
SN_API void sn_nav_default_config(sn_nav_config *out_config);

/// Builds a tiled navmesh from a triangle soup, tiles in parallel. Inputs must stay alive until
/// the task finishes. Result [0]: navmesh handle; [1]: tile count.
SN_API int32_t sn_nav_build(const float *vertices, uint32_t vertex_count, const int32_t *triangles, uint32_t triangle_count, const sn_nav_config *config);
/// Loads a navmesh serialized by sn_nav_serialize or the stride-tools bake. Returns a handle.
SN_API int32_t sn_nav_load(const void *data, uint32_t size);
/// Serializes a navmesh. Result buffer is allocated with sn_alloc; the caller frees it.
SN_API int32_t sn_nav_serialize(int32_t navmesh, void **out_data, uint32_t *out_size);
SN_API void sn_nav_destroy(int32_t navmesh);
/// Straight path between two points (corners written as xyz). Returns the corner count.
SN_API int32_t sn_nav_find_path(int32_t navmesh, const float *start3, const float *end3, const float *search_extents3, float *out_points, uint32_t max_points);

SN_API int32_t sn_crowd_create(int32_t navmesh, uint32_t max_agents, float max_agent_radius);
SN_API void sn_crowd_destroy(int32_t crowd);
/// Returns the agent index.
SN_API int32_t sn_crowd_add_agent(int32_t crowd, const float *position3, float radius, float height, float max_speed, float max_acceleration);
SN_API void sn_crowd_remove_agent(int32_t crowd, int32_t agent);
SN_API int32_t sn_crowd_request_move(int32_t crowd, int32_t agent, const float *target3);
/// Moves the agents, then writes their positions into the positions buffer.
SN_API void sn_crowd_update(int32_t crowd, float delta_time);
/// The crowd's positions buffer: xyz per agent slot (NaN for empty slots), room for max_agents. It stays
/// at this address until the crowd is destroyed; add_agent, remove_agent and update keep it current.
SN_API const float *sn_crowd_positions(int32_t crowd);

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
