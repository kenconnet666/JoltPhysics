// 物理侧模块（stride-native-physics）的 C API：Jolt。物体可以绑定运行时模块的变换存储（Runtime.h）。
#pragma once

#include <StrideNative/Runtime.h>

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
