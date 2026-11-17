// stride-tools C API: offline asset pipeline functions, used from the editor's Node backend.
// Heavy operations return tasks so several assets can be processed in parallel.
#pragma once

#include <StrideNative/Core.h>
#include <StrideNative/Runtime.h>

// ---------------------------------------------------------------------------------------------
// Scene import (glTF/GLB via cgltf, FBX via ufbx). Meshes are triangulated, indexed, in the
// mesh's own space, right-handed Y-up, meters.
// ---------------------------------------------------------------------------------------------

enum
{
	ST_FORMAT_AUTO = 0,
	ST_FORMAT_GLTF = 1,				///< .gltf with embedded (data URI) buffers, or .glb
	ST_FORMAT_FBX = 2,
};

typedef struct st_mesh_info
{
	uint32_t vertex_count;
	uint32_t index_count;
	int32_t has_normals;
	int32_t has_uvs;
} st_mesh_info;

/// Parses a scene. data must stay alive until the task finishes. Result [0]: scene handle,
/// [1]: mesh count.
SN_API int32_t st_scene_load(const void *data, uint32_t size, int32_t format);
SN_API void st_scene_destroy(int32_t scene);
SN_API int32_t st_scene_mesh_info(int32_t scene, uint32_t mesh, st_mesh_info *out_info);
/// Copies a mesh; any output may be null. positions/normals xyz, uvs xy, indices uint32.
SN_API int32_t st_scene_mesh_copy(int32_t scene, uint32_t mesh, float *positions, float *normals, float *uvs, uint32_t *indices);

// ---------------------------------------------------------------------------------------------
// Geometry processing
// ---------------------------------------------------------------------------------------------

/// MikkTSpace tangents, one xyzw per index (corner); weld afterwards if needed.
SN_API int32_t st_generate_tangents(const float *positions, const float *normals, const float *uvs, const uint32_t *indices, uint32_t index_count, float *out_tangents4);

/// Vertex cache plus overdraw optimization, in place.
SN_API int32_t st_meshopt_optimize(uint32_t *indices, uint32_t index_count, const float *positions, uint32_t vertex_count, uint32_t position_stride);
/// Returns the new index count; out_error receives the relative error reached.
SN_API int32_t st_meshopt_simplify(uint32_t *out_indices, const uint32_t *indices, uint32_t index_count, const float *positions, uint32_t vertex_count, uint32_t position_stride, uint32_t target_index_count, float target_error, float *out_error);
SN_API uint32_t st_meshopt_encode_vertex_bound(uint32_t vertex_count, uint32_t vertex_size);
SN_API uint32_t st_meshopt_encode_index_bound(uint32_t index_count, uint32_t vertex_count);
/// Encode for SN_MESHOPT_MODE_ATTRIBUTES / SN_MESHOPT_MODE_TRIANGLES. Return the encoded size (0 on failure).
SN_API uint32_t st_meshopt_encode_vertices(void *dst, uint32_t dst_capacity, const void *vertices, uint32_t vertex_count, uint32_t vertex_size);
SN_API uint32_t st_meshopt_encode_indices(void *dst, uint32_t dst_capacity, const uint32_t *indices, uint32_t index_count);

/// Lightmap UV unwrap (xatlas). Inputs must stay alive until the task finishes.
/// Outputs: [0] uint32 original-vertex index per new vertex, [1] float2 uv in [0,1] per new
/// vertex, [2] uint32 new indices. Results: [0] atlas width, [1] atlas height, [2] new vertex count.
SN_API int32_t st_uv_unwrap(const float *positions, const float *normals, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count, float texels_per_unit);

/// Convex decomposition (V-HACD) for physics. Inputs must stay alive until the task finishes.
/// Outputs: [0] int32 pairs (vertex count, triangle count) per hull, [1] float3 vertices of all
/// hulls, [2] uint32 triangle indices, local to their hull. Result [0]: hull count.
SN_API int32_t st_convex_decompose(const float *positions, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count, uint32_t max_hulls, uint32_t voxel_resolution, uint32_t max_vertices_per_hull);

// ---------------------------------------------------------------------------------------------
// Textures and fonts
// ---------------------------------------------------------------------------------------------

enum
{
	ST_BASIS_UASTC = 0,				///< High quality, larger; good for normal maps
	ST_BASIS_ETC1S = 1,				///< Small, lower quality
};

enum
{
	ST_BASIS_FLAG_SRGB = 1,
	ST_BASIS_FLAG_MIPMAPS = 2,
};

/// Encodes an RGBA8 image into a KTX2 file. rgba must stay alive until the task finishes.
/// quality 0-100, effort 0-10. Output [0]: the .ktx2 file.
SN_API int32_t st_basis_encode(const uint8_t *rgba, uint32_t width, uint32_t height, int32_t mode, int32_t flags, int32_t quality, int32_t effort);

typedef struct st_glyph_metrics
{
	uint32_t codepoint;
	uint32_t width;					///< Bitmap size in pixels, including the distance range border
	uint32_t height;
	float bearing_x;				///< Pen position to bitmap left edge, pixels
	float bearing_y;				///< Baseline to bitmap top edge, pixels (positive up)
	float advance;					///< Pixels
} st_glyph_metrics;

/// Multi-channel signed distance field glyphs (stb_truetype outlines, msdfgen), in parallel.
/// font and codepoints must stay alive until the task finishes. Outputs: [0] st_glyph_metrics per
/// codepoint, then one RGB8 bitmap per codepoint (empty for glyphs without outline).
SN_API int32_t st_font_msdf(const void *font, uint32_t font_size, const uint32_t *codepoints, uint32_t codepoint_count, float pixel_size, float distance_range);

// ---------------------------------------------------------------------------------------------
// Animation building (ozz-animation offline)
// ---------------------------------------------------------------------------------------------

typedef struct st_joint
{
	int32_t parent;					///< -1 for roots; parents must come before children
	float translation[3];
	float rotation[4];				///< xyzw
	float scale[3];
} st_joint;

/// Builds an ozz skeleton archive. names: joint_count NUL-terminated strings back to back (may
/// be null). Output buffer is allocated with sn_alloc; the caller frees it.
SN_API int32_t st_anim_build_skeleton(const st_joint *joints, uint32_t joint_count, const char *names, void **out_data, uint32_t *out_size);

/// Builds an ozz animation archive. For each joint, key_counts holds (translation, rotation,
/// scale) counts; keys are packed per joint in that order: translation = time,x,y,z;
/// rotation = time,x,y,z,w; scale = time,x,y,z. Times are in seconds.
SN_API int32_t st_anim_build_animation(float duration, uint32_t joint_count, const uint32_t *key_counts, const float *keys, void **out_data, uint32_t *out_size);

// ---------------------------------------------------------------------------------------------
// Navigation baking
// ---------------------------------------------------------------------------------------------

/// Same tiled parallel build as sn_nav_build, but outputs the serialized navmesh for
/// sn_nav_load. Output [0]: navmesh data. Result [0]: tile count.
SN_API int32_t st_nav_bake(const float *vertices, uint32_t vertex_count, const int32_t *triangles, uint32_t triangle_count, const sn_nav_config *config);
