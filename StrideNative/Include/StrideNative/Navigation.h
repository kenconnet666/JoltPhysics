// 导航侧模块（stride-native-navigation）的 C API：Recast/Detour。sn_nav_config 和 sn_nav_default_config 也给工具模块的烘焙用。
#pragma once

#include <StrideNative/Core.h>

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
/// How carefully agents avoid each other (Recast demo presets): 0 low, 1 medium (default), 2 good, 3 high. Applies to
/// every agent; higher costs more per update.
SN_API int32_t sn_crowd_set_avoidance_quality(int32_t crowd, int32_t quality);
/// Velocities (xyz per agent slot, 0 for empty slots), kept like the positions buffer: which way each agent walks.
SN_API const float *sn_crowd_velocities(int32_t crowd);
