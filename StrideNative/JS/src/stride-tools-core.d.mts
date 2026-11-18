// Emscripten 生成的 stride-tools-core.mjs 的类型：模块工厂和这个模块导出的 C 函数（Tools.h）。
import type { CoreFactory } from './native-module.js';

export type ToolsExport =
	| 'st_scene_load' | 'st_scene_mesh_info' | 'st_scene_material_name' | 'st_scene_mesh_copy' | 'st_scene_destroy' | 'st_generate_tangents'
	| 'st_meshopt_optimize' | 'st_meshopt_simplify' | 'st_meshopt_encode_vertex_bound' | 'st_meshopt_encode_vertices'
	| 'st_meshopt_encode_index_bound' | 'st_meshopt_encode_indices' | 'st_uv_unwrap' | 'st_convex_decompose'
	| 'st_image_decode' | 'st_cubemap_prefilter_ggx' | 'st_cubemap_sh_lambert' | 'st_basis_encode' | 'st_font_msdf' | 'st_anim_build_skeleton' | 'st_anim_build_animation'
	| 'sn_nav_default_config' | 'st_nav_bake';

declare const factory: CoreFactory<ToolsExport>;
export default factory;
