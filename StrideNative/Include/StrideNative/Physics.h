// 物理侧模块（stride-native-physics）的 C API：Stride.BepuPhysics 形状的一层，内核是 Jolt。
// 物体可以绑定运行时模块的变换存储（Runtime.h），位姿直接读写共享的变换数组。
#pragma once

#include <StrideNative/Runtime.h>

// ---------------------------------------------------------------------------------------------
// 世界
// ---------------------------------------------------------------------------------------------

/// 仿真设置（Stride 的 BepuSimulation）。
typedef struct sn_physics_settings
{
	float gravity[3];				///< PoseGravity
	float linear_damping;			///< PoseLinearDamping：每秒衰减的比例
	float angular_damping;			///< PoseAngularDamping
	/// 碰撞矩阵（CollisionMatrix）：collision_masks[a] 的第 b 位为 1 表示层 a 和层 b 的物体相互碰撞，要对称
	uint32_t collision_masks[32];
} sn_physics_settings;

/// 建一个在共享任务系统上步进的世界，返回句柄。transforms 是物体可以绑定的变换存储（0 表示不用），它要比世界
/// 活得久，绑定的槽位在物体存在期间不能释放。settings 为空时用 Stride 的默认值（重力 -9.8、阻尼 0.05、全部层相互碰撞）。
SN_API int32_t sn_physics_create(uint32_t max_bodies, int32_t transforms, const sn_physics_settings *settings);
SN_API void sn_physics_destroy(int32_t world);

// ---------------------------------------------------------------------------------------------
// 形状和物体
// ---------------------------------------------------------------------------------------------

enum
{
	SN_SHAPE_BOX = 0,				///< size：边长（Stride 的 BoxCollider.Size）
	SN_SHAPE_SPHERE = 1,			///< size[0]：半径
	SN_SHAPE_CAPSULE = 2,			///< size[0]：半径，size[1]：两端球心的距离（CapsuleCollider.Length），沿本地 Y
	SN_SHAPE_CYLINDER = 3,			///< size[0]：半径，size[1]：长度，沿本地 Y
	SN_SHAPE_MESH = 4,				///< mesh：sn_physics_create_mesh 的结果；按实体的世界缩放缩放（Stride 的 MeshCollider）
	SN_SHAPE_CONVEX_HULL = 5,		///< mesh：sn_physics_create_hull 的结果；同网格按实体的世界缩放缩放（Stride 的 ConvexHullCollider）
};

/// 复合碰撞体里的一个形状（Stride 的 ColliderBase 加 CompoundCollider 的本地位姿）。
typedef struct sn_shape
{
	uint32_t type;					///< SN_SHAPE_*
	float position[3];				///< 本地位置
	float rotation[4];				///< 本地旋转，xyzw
	float size[3];
	int32_t mesh;					///< SN_SHAPE_MESH、SN_SHAPE_CONVEX_HULL 用
	float mass;						///< 动态物体和角色的质量是各形状的和
} sn_shape;

enum
{
	SN_MOTION_STATIC = 0,			///< StaticComponent
	SN_MOTION_DYNAMIC = 1,			///< BodyComponent
	SN_MOTION_KINEMATIC = 2,		///< BodyComponent.Kinematic
	SN_MOTION_CHARACTER = 3,		///< CharacterComponent：不转动的动态刚体加 Stride 的角色逻辑
};

enum
{
	SN_BODY_INTERPOLATED = 1,		///< InterpolationMode.Interpolated：sn_physics_interpolate 写回两步之间的位姿
	SN_BODY_NO_GRAVITY = 2,			///< Gravity = false
	SN_BODY_NEVER_SLEEP = 4,		///< SleepThreshold 为负：一直醒着（Jolt 的睡眠阈值是全局的，按物体只能开关）
};

/// 角色参数（Stride 的 CharacterComponent），SN_MOTION_CHARACTER 的物体用。
typedef struct sn_character
{
	float maximum_horizontal_force;
	float maximum_vertical_force;
	float jump_velocity;			///< JumpForce（Stride 的数据别名 JumpSpeed）
	float slope_angle;				///< 度
	float air_control_force_scale;
	float air_control_scale;
} sn_character;

typedef struct sn_body
{
	uint32_t motion;				///< SN_MOTION_*
	uint32_t flags;					///< SN_BODY_*
	/// 绑定的变换槽位，0 表示不绑定。绑定后初始位姿取槽位当前的世界位姿（缩放只作用到网格形状），
	/// 动态物体和角色每步把位姿写回槽位（有父节点时换算到父节点空间）。
	int32_t transform;
	float position[3];				///< 没有绑定变换时的位姿
	float rotation[4];
	uint32_t first_shape;			///< shapes 表里的下标
	uint32_t shape_count;
	uint32_t layer;					///< CollisionLayer，0..31
	float friction;					///< FrictionCoefficient
	uint32_t character;				///< SN_MOTION_CHARACTER 时 characters 表里的下标
} sn_body;

/// 三角网格形状，世界内的物体共用。positions 是 xyz，indices 每三个一个三角形（逆时针为正面）。返回网格句柄。
SN_API int32_t sn_physics_create_mesh(int32_t world, const float *positions, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count);
/// 凸包形状（Jolt 由点算出凸包），世界内的物体共用；和网格共用句柄空间。positions 是 xyz。返回句柄。
SN_API int32_t sn_physics_create_hull(int32_t world, const float *positions, uint32_t point_count);
/// 一次加入 count 个物体（Jolt 成批插进宽相），返回第一个物体的编号；编号按加入顺序递增。
SN_API int32_t sn_physics_add_bodies(int32_t world, const sn_body *bodies, uint32_t count, const sn_shape *shapes, uint32_t shape_count,
	const sn_character *characters, uint32_t character_count);

// ---------------------------------------------------------------------------------------------
// 步进
// ---------------------------------------------------------------------------------------------

/// 步进一个固定步长：先跑角色逻辑，再 Jolt 步进，再更新角色的支撑状态，最后写回位姿（插值的物体记下前后两步）。
SN_API int32_t sn_physics_step(int32_t world, float delta_time, int32_t collision_steps);
/// 插值的物体写回 alpha（0 是上一步，1 是这一步）处的位姿。
SN_API int32_t sn_physics_interpolate(int32_t world, float alpha);
/// 每个物体的位置 xyz，按编号排，能放 max_bodies 个；地址在世界销毁前不变，add_bodies 和 step 原地改写。
SN_API const float *sn_physics_positions(int32_t world);

// ---------------------------------------------------------------------------------------------
// 动态和运动学物体（Stride 的 BodyComponent）。body 是物体编号；成功返回 SN_OK。
// ---------------------------------------------------------------------------------------------

/// 在动态和运动学之间切换（Stride 的 Kinematic）：只对按动态或运动学加入的物体有效。
SN_API int32_t sn_physics_body_set_kinematic(int32_t world, int32_t body, int32_t kinematic);
/// 唤醒或让它睡下（Stride 的 Awake）。
SN_API int32_t sn_physics_body_set_awake(int32_t world, int32_t body, int32_t awake);
/// 1 醒着，0 睡着，负数是错误。
SN_API int32_t sn_physics_body_awake(int32_t world, int32_t body);
/// 瞬移到世界位姿（Stride 的 Teleport），并唤醒；绑定了变换的立即写回槽位，插值也从这里重新开始。
SN_API int32_t sn_physics_body_teleport(int32_t world, int32_t body, const float *position, const float *rotation);
/// 读线速度和角速度（世界空间，各 xyz）；不要的传 null。
SN_API int32_t sn_physics_body_velocity(int32_t world, int32_t body, float *linear, float *angular);
/// 设线速度和角速度；传 null 的不变。
SN_API int32_t sn_physics_body_set_velocity(int32_t world, int32_t body, const float *linear, const float *angular);
/// 在质心加冲量（Stride 的 ApplyLinearImpulse、ApplyAngularImpulse）；传 null 的不加。运动学物体忽略。
SN_API int32_t sn_physics_body_apply_impulse(int32_t world, int32_t body, const float *linear, const float *angular);
/// 受不受重力（Stride 的 Gravity）。
SN_API int32_t sn_physics_body_set_gravity(int32_t world, int32_t body, int32_t enabled);

// ---------------------------------------------------------------------------------------------
// 角色
// ---------------------------------------------------------------------------------------------

/// 设下一步的移动：target 是世界 X、Z 方向的目标速度（Stride 的 MoveVector · Speed），jump 不为 0 时试着起跳
/// （同 TryJump：只在站稳时生效，下一步用掉）。
SN_API int32_t sn_physics_character_move(int32_t world, int32_t body, float target_x, float target_z, int32_t jump);
/// 角色是否站在可以站立的地面上（Stride 的 IsGrounded）：1 是，0 否，负数是错误。
SN_API int32_t sn_physics_character_grounded(int32_t world, int32_t body);

// ---------------------------------------------------------------------------------------------
// 查询
// ---------------------------------------------------------------------------------------------

typedef struct sn_hit
{
	float distance;					///< 沿方向的距离
	float point[3];
	float normal[3];
	int32_t body;					///< 命中的物体编号
} sn_hit;

/// 射线（Stride 的 RayCast）：direction 不用归一化，max_distance 是米。mask 的第 n 位为 1 表示层 n 的物体参与。
/// 返回 1 命中（写 hit），0 没有，负数是错误。
SN_API int32_t sn_physics_ray_cast(int32_t world, const float *origin, const float *direction, float max_distance, uint32_t mask, sn_hit *hit);
enum
{
	/// 球体扫掠忽略起点就和球重叠的物体（默认它们算距离 0 的命中，同 Bepu）。比如第三人称相机：贴着旁边的箱子时
	/// 不该把相机拉到最近；要防止相机钻进正后方的墙，配一条射线。
	SN_CAST_IGNORE_INITIAL_OVERLAP = 1,
};

/// 球体扫掠（Stride 的 SweepCast 加 Sphere），同 sn_physics_ray_cast；flags 是 SN_CAST_*。
SN_API int32_t sn_physics_sphere_cast(int32_t world, float radius, const float *origin, const float *direction, float max_distance, uint32_t mask, uint32_t flags, sn_hit *hit);
