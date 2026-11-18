// Emscripten 生成的 stride-native-core.mjs 的类型：模块工厂和运行时导出的 C 函数（Runtime.h、Plugin.h），加上内置
// 侧模块的（Physics.h、Texture.h、Navigation.h）：侧模块加载后它们的导出挂在同一个模块对象上，没加载时不存在。
import type { CoreFactory } from './native-module.js';

export type RuntimeExport =
	| 'sn_transforms_create' | 'sn_transforms_destroy' | 'sn_transforms_columns' | 'sn_transforms_alloc'
	| 'sn_transforms_free' | 'sn_transforms_set_parent' | 'sn_transforms_update' | 'sn_transforms_slot_limit' | 'sn_transforms_cull'
	| 'sn_physics_create' | 'sn_physics_destroy' | 'sn_physics_add_boxes' | 'sn_physics_step' | 'sn_physics_positions'
	| 'sn_meshopt_decode' | 'sn_meshopt_decode_batch' | 'sn_ktx2_get_info' | 'sn_ktx2_transcode'
	| 'sn_nav_default_config' | 'sn_nav_build' | 'sn_nav_load' | 'sn_nav_serialize' | 'sn_nav_find_path' | 'sn_nav_destroy'
	| 'sn_crowd_create' | 'sn_crowd_destroy' | 'sn_crowd_add_agent' | 'sn_crowd_remove_agent' | 'sn_crowd_request_move'
	| 'sn_crowd_update' | 'sn_crowd_positions'
	| 'sn_anim_load_skeleton' | 'sn_anim_destroy_skeleton' | 'sn_anim_skeleton_joint_count' | 'sn_anim_load_animation'
	| 'sn_anim_destroy_animation' | 'sn_anim_animation_duration' | 'sn_anim_instance_create' | 'sn_anim_instance_destroy'
	| 'sn_anim_instance_model_matrices' | 'sn_anim_sample_batch' | 'sn_plugin_abi_version';

declare const factory: CoreFactory<RuntimeExport>;
export default factory;
