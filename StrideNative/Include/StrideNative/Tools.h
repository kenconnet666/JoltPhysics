// stride-tools C API: offline asset pipeline functions, used from the editor's Node backend.
// Heavy operations return tasks so several assets can be processed in parallel.
#pragma once

#include <StrideNative/Core.h>
#include <StrideNative/Navigation.h>

// ---------------------------------------------------------------------------------------------
// Scene import (glTF/GLB via cgltf, FBX via ufbx). Meshes are triangulated and indexed, one per
// node instance and material (like Stride's model import), with the node's world transform baked in:
// model space, right-handed Y-up, meters.
// ---------------------------------------------------------------------------------------------

enum
{
	ST_FORMAT_AUTO = 0,
	ST_FORMAT_GLTF = 1,				///< .gltf with embedded (data URI) buffers, or .glb
	ST_FORMAT_FBX = 2,
	ST_FORMAT_OBJ = 3,				///< Wavefront OBJ（ufbx 读，不读 .mtl，材质只有 usemtl 的名字；不换坐标轴和单位，同 Stride 的 Assimp 导入）
};

enum
{
	ST_NO_MATERIAL = 0xffffffffu,	///< st_mesh_info.material of a mesh without a material
};

typedef struct st_mesh_info
{
	uint32_t vertex_count;
	uint32_t index_count;
	int32_t has_normals;
	int32_t has_uvs;
	uint32_t material;				///< index into the scene's materials, or ST_NO_MATERIAL
	uint32_t node;					///< the node instancing the mesh (index into the scene's nodes)
	uint32_t joint_count;			///< skinned meshes: joints influencing it (st_scene_mesh_skin); 0 otherwise
	uint32_t morph_target_count;	///< morph targets (glTF targets, FBX blend channels; st_scene_mesh_morph); 0 if none
	int32_t morph_has_normals;		///< whether the morph targets have normal deltas
} st_mesh_info;

/// A node of the scene hierarchy at rest (the file's default pose).
typedef struct st_node_info
{
	int32_t parent;					///< -1 for the root; parents come before their children
	float translation[3];			///< local transform
	float rotation[4];				///< xyzw
	float scale[3];
	float world[16];				///< column-major node-to-model matrix
} st_node_info;

/// Parses a scene. data must stay alive until the task finishes. Result [0]: scene handle,
/// [1]: mesh count, [2]: material count, [3]: node count, [4]: animation count.
/// FBX's root node is named "RootNode" (like Stride's importer); nodes keep the file's order.
SN_API int32_t st_scene_load(const void *data, uint32_t size, int32_t format);
SN_API void st_scene_destroy(int32_t scene);
SN_API int32_t st_scene_mesh_info(int32_t scene, uint32_t mesh, st_mesh_info *out_info);
/// Name of a material (UTF-8), valid until the scene is destroyed; null for a bad index.
SN_API const char *st_scene_material_name(int32_t scene, uint32_t material);
/// Copies a mesh; any output may be null. positions/normals xyz, uvs xy, indices uint32.
SN_API int32_t st_scene_mesh_copy(int32_t scene, uint32_t mesh, float *positions, float *normals, float *uvs, uint32_t *indices);

SN_API int32_t st_scene_node_info(int32_t scene, uint32_t node, st_node_info *out_info);
/// Name of a node (UTF-8), valid until the scene is destroyed; null for a bad index.
SN_API const char *st_scene_node_name(int32_t scene, uint32_t node);
/// Skin of a skinned mesh (st_mesh_info.joint_count > 0); any output may be null.
/// joint_nodes: joint_count node indices. inverse_binds: joint_count column-major 4x4 matrices taking the
/// mesh's (baked, model-space) vertices to the joint's space at bind time, so a joint's skinning matrix is
/// its current node-to-model matrix times its inverse bind. joints/weights: 4 per vertex, indices into
/// joint_nodes sorted by decreasing weight, weights normalized (unused slots have weight 0).
SN_API int32_t st_scene_mesh_skin(int32_t scene, uint32_t mesh, uint32_t *joint_nodes, float *inverse_binds, uint16_t *joints, float *weights);

/// Name of a morph target of a mesh (UTF-8: glTF's extras.targetNames, FBX's blend channel), valid until the scene
/// is destroyed; null for a bad index.
SN_API const char *st_scene_mesh_morph_name(int32_t scene, uint32_t mesh, uint32_t target);
/// Deltas of a morph target, xyz per vertex in the mesh's (baked, model) space; any output may be null. normal_deltas
/// stays untouched when the targets have no normals (st_mesh_info.morph_has_normals).
SN_API int32_t st_scene_mesh_morph(int32_t scene, uint32_t mesh, uint32_t target, float *position_deltas, float *normal_deltas);

/// Name of an animation (glTF animation, FBX animation stack), valid until the scene is destroyed; null for a bad index.
SN_API const char *st_scene_animation_name(int32_t scene, uint32_t animation);
/// Bakes an animation into linearly interpolated local-transform keys for every node, resampled at sample_rate
/// keys per second, times starting at 0 (FBX: from the start of playback; glTF: from 0, ending at the last key).
/// Returns a task; outputs: [0] key counts (uint32, translation, rotation, scale per node), [1] keys in
/// st_anim_build_animation's layout, [2] the duration in seconds (one float), [3] morph weight key counts (uint32,
/// one per target of every mesh with morph targets, in mesh then target order; 0 when the animation leaves the
/// weight alone), [4] morph weight keys (time, weight). Nodes the animation leaves alone get one key of their rest transform.
SN_API int32_t st_scene_animation_bake(int32_t scene, uint32_t animation, float sample_rate);

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

/// Decodes a PNG, JPEG, TGA or DDS file (the largest mip: BC1-BC5, BC7, 24/32-bit RGB) into RGBA8. data must stay alive until the task finishes.
/// Result [0]: width, [1]: height, [2]: channels in the file. Output [0]: the pixels, rows top-down.
SN_API int32_t st_image_decode(const void *data, uint32_t size);

/// Resamples RGBA8 pixels (rows top-down) to out_width x out_height with a Lanczos3 filter on the stored values, edges clamped
/// (Stride's TextureTool.Resize with Filter.Rescaling.Lanczos3). rgba must stay alive until the task finishes.
/// Output [0]: the resampled pixels.
SN_API int32_t st_image_resize(const uint8_t *rgba, uint32_t width, uint32_t height, uint32_t out_width, uint32_t out_height);

/// Cubemaps below are RGBA32F, faces in the order +X -X +Y -Y +Z -Z, rows top-down (the D3D/WebGPU layout).

/// Prefilters a radiance cubemap for GGX environment lighting like Stride's RadiancePrefilteringGGX:
/// level 0 of the out_size cube is the source resampled, level i is filtered with roughness i / (levels - 1)
/// using samples GGX importance samples. faces is level 0 of a size x size cube (mipmaps are generated);
/// both sizes must be powers of two. faces must stay alive until the task finishes.
/// Output [0]: every level from out_size down to 1, each 6 faces, concatenated.
SN_API int32_t st_cubemap_prefilter_ggx(const float *faces, uint32_t size, uint32_t out_size, uint32_t samples);

/// Projects a radiance cubemap onto order-3 spherical harmonics with the Lambertian cosine lobe, like Stride's
/// LambertianPrefilteringSH; the coefficients are already multiplied by the basis constants, so evaluating
/// them only needs the polynomials. faces must stay alive until the task finishes.
/// Output [0]: 9 RGB coefficients (27 floats).
SN_API int32_t st_cubemap_sh_lambert(const float *faces, uint32_t size);

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

/// A font's vertical metrics and names (font units). font_count is the number of fonts in a collection (1 for a
/// plain .ttf/.otf); index picks one. The hhea, OS/2 typo and OS/2 win metrics are all given: renderers differ in
/// which they use (FreeType takes hhea unless use_typo_metrics). descent and typo_descent are negative, win_descent positive.
typedef struct st_font_description
{
	int32_t font_count;
	int32_t glyph_count;
	int32_t units_per_em;
	int32_t bounds[4];				///< head xMin yMin xMax yMax
	int32_t ascent;					///< hhea
	int32_t descent;
	int32_t line_gap;
	int32_t use_typo_metrics;		///< OS/2 fsSelection bit 7
	int32_t typo_ascent;
	int32_t typo_descent;
	int32_t typo_line_gap;
	int32_t win_ascent;
	int32_t win_descent;
	int32_t x_height;				///< OS/2 version 2+, else 0
	int32_t cap_height;
	int32_t head_flags;				///< head flags; bit 3 asks rasterizers to round the ppem to an integer (FreeType does when hinting)
	char family[128];				///< name ID 1, UTF-8 ("Arial")
	char subfamily[128];			///< name ID 2 ("Bold Italic")
	char typographic_family[128];	///< name IDs 16 and 17, empty when absent
	char typographic_subfamily[128];
} st_font_description;

/// Describes the font at index of a font file or collection (synchronous).
SN_API int32_t st_font_describe(const void *font, uint32_t font_size, int32_t index, st_font_description *out_description);

/// Every codepoint the font maps (U+0000 to U+10FFFF) and the legacy kern table. font must stay alive until the task
/// finishes. Outputs: [0] uint32 codepoints ascending, [1] uint32 glyph index per codepoint, [2] kern pairs as int32
/// (glyph1, glyph2, advance in font units), sorted by glyph1 then glyph2.
SN_API int32_t st_font_codepoints(const void *font, uint32_t font_size, int32_t index);

typedef struct st_glyph_outline
{
	uint32_t glyph;
	int32_t advance;				///< Font units
	int32_t left_bearing;
	int32_t bounds[4];				///< x0 y0 x1 y1 of the outline, font units, y up; zero without outline
	uint32_t curve_offset;			///< First curve in output [1]
	uint32_t curve_count;
} st_glyph_outline;

/// Glyph outlines as quadratic Bezier curves for the vector text renderer, in parallel. Cubic (CFF) segments are split
/// into quadratics within tolerance font units; a line is a quadratic whose control point equals its end point.
/// font and glyphs must stay alive until the task finishes. Outputs: [0] st_glyph_outline per glyph, [1] float curves
/// (x1 y1 x2 y2 x3 y3, font units, y up), each glyph's contours closed.
SN_API int32_t st_font_outlines(const void *font, uint32_t font_size, int32_t index, const uint32_t *glyphs, uint32_t glyph_count, float tolerance);

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
