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
/// 运行中改重力（Stride 的 BepuSimulation.PoseGravity）；睡着的物体要被碰到或唤醒才受新的重力。
SN_API int32_t sn_physics_set_gravity(int32_t world, const float *gravity);

// ---------------------------------------------------------------------------------------------
// 形状和物体
// ---------------------------------------------------------------------------------------------

enum
{
	SN_SHAPE_BOX = 0,				///< size：边长（Stride 的 BoxCollider.Size）
	SN_SHAPE_SPHERE = 1,			///< size[0]：半径
	SN_SHAPE_CAPSULE = 2,			///< size[0]：半径，size[1]：两端球心的距离（CapsuleCollider.Length），沿本地 Y
	SN_SHAPE_CYLINDER = 3,			///< size[0]：半径，size[1]：长度，沿本地 Y
	SN_SHAPE_MESH = 4,				///< mesh：sn_physics_create_mesh 的结果；按实体的世界缩放缩放（Stride 的 MeshCollider）；动态物体上只能单独用，质量属性按封闭网格积分
	SN_SHAPE_CONVEX_HULL = 5,		///< mesh：sn_physics_create_hull 的结果；同网格按实体的世界缩放缩放（Stride 的 ConvexHullCollider）
	SN_SHAPE_EMPTY = 6,				///< 没有形状、不碰撞（Stride 的 EmptyCollider）；只能单独用，动态时转动惯量同 Bepu 的半径 1 的球
	SN_SHAPE_HEIGHTFIELD = 7,		///< mesh：sn_physics_create_heightfield 的结果（地形的一块）；只能用在静态物体上
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
	SN_BODY_SENSOR = 8,				///< 只检测、不产生碰撞响应（Stride 的 Trigger，或 ContactEventHandler.NoContactResponse）
	SN_BODY_CONTACT_EVENTS = 16,	///< 报告开始和结束接触（Stride 的 ContactEventHandler）
	SN_BODY_TOUCHING_EVENTS = 32,	///< 另外每步报正在接触的深度（Stride 的 IContactHandler.OnTouching），含 SN_BODY_CONTACT_EVENTS
	SN_BODY_CONTINUOUS = 64,		///< 连续碰撞检测：沿这一步的位移扫过去找碰撞，快的小东西不穿过薄墙（Stride 的 ContinuousDetectionMode.Continuous，Jolt 的 LinearCast）
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
	double position[3];				///< 没有绑定变换时的位姿（世界位置是双精度）
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
/**
 * 高度场形状（地形的一块碰撞）：sample_count × sample_count 个高度（米，按行 z 排），格距 spacing 米；形状的本地原点是第一个
 * 样本，x、z 往正方向。返回的句柄用在 SN_SHAPE_HEIGHTFIELD 的 mesh 上。
 */
SN_API int32_t sn_physics_create_heightfield(int32_t world, const float *heights, uint32_t sample_count, float spacing);
/** 网格、凸包、高度场不再用来建新物体了：句柄作废（之后会复用）；已经用着它的物体不受影响，物体都删了形状才真的释放。 */
SN_API void sn_physics_release_shape(int32_t world, int32_t shape);
/// 移除物体（实体销毁时）：从世界里拿走并释放，编号留给以后加的物体复用；和它有关的接触不再报告。移除过的忽略。
SN_API int32_t sn_physics_remove_bodies(int32_t world, const int32_t *bodies, uint32_t count);
/// 一次加入 count 个物体（Jolt 成批插进宽相），每个物体的编号写进 out_bodies（可以是 null），返回第一个物体的编号。
/// 先复用移除过的物体的编号，所以编号不一定连续；没有移除过的物体时按加入顺序递增。同时存在的物体不超过 max_bodies。
SN_API int32_t sn_physics_add_bodies(int32_t world, const sn_body *bodies, uint32_t count, const sn_shape *shapes, uint32_t shape_count,
	const sn_character *characters, uint32_t character_count, int32_t *out_bodies);

// ---------------------------------------------------------------------------------------------
// 步进
// ---------------------------------------------------------------------------------------------

/// 步进一个固定步长：先跑角色逻辑，再 Jolt 步进，再更新角色的支撑状态，最后写回位姿（插值的物体记下前后两步）。
SN_API int32_t sn_physics_step(int32_t world, float delta_time, int32_t collision_steps);
/// 插值的物体写回 alpha（0 是上一步，1 是这一步）处的位姿。
SN_API int32_t sn_physics_interpolate(int32_t world, float alpha);
/// 每个物体的位置 xyz，按编号排，能放 max_bodies 个；地址在世界销毁前不变，add_bodies 和 step 原地改写。
SN_API const double *sn_physics_positions(int32_t world);

// ---------------------------------------------------------------------------------------------
// 动态和运动学物体（Stride 的 BodyComponent）。body 是物体编号；成功返回 SN_OK。
// ---------------------------------------------------------------------------------------------

/// 在动态和运动学之间切换（Stride 的 Kinematic）：只对按动态或运动学加入的物体有效。
SN_API int32_t sn_physics_body_set_kinematic(int32_t world, int32_t body, int32_t kinematic);
/// 唤醒或让它睡下（Stride 的 Awake）。
SN_API int32_t sn_physics_body_set_awake(int32_t world, int32_t body, int32_t awake);
/// 1 醒着，0 睡着，负数是错误。
SN_API int32_t sn_physics_body_awake(int32_t world, int32_t body);
/// 瞬移到世界位姿（Stride 的 Teleport），并唤醒；绑定了变换的立即写回槽位，插值也从这里重新开始。角色也可以（转身改朝向，
/// Stride 的 CharacterComponent.Orientation），之后的 MoveVector 按新的朝向换到世界。
SN_API int32_t sn_physics_body_teleport(int32_t world, int32_t body, const double *position, const float *rotation);
/// 只改朝向（Stride 的 SetTargetPose(Quaternion)、CharacterComponent.Orientation），并唤醒：位置和它的插值不动，所以可以每帧调
/// （鼠标转视角带着角色转）。绑定了变换的立即写回旋转。角色之后的 MoveVector 按新的朝向换到世界。
SN_API int32_t sn_physics_body_set_rotation(int32_t world, int32_t body, const float *rotation);
/// 读线速度和角速度（世界空间，各 xyz）；不要的传 null。角色也可以读。
SN_API int32_t sn_physics_body_velocity(int32_t world, int32_t body, float *linear, float *angular);
/// 设线速度和角速度；传 null 的不变。角色也可以设（比如瞬移后清零），之后每步的角色逻辑从这个速度出发追移动目标。
SN_API int32_t sn_physics_body_set_velocity(int32_t world, int32_t body, const float *linear, const float *angular);
/// 在质心加冲量（Stride 的 ApplyLinearImpulse、ApplyAngularImpulse）；传 null 的不加。运动学物体忽略；角色受（击退），水平速度之后由角色逻辑拉回移动目标。
SN_API int32_t sn_physics_body_apply_impulse(int32_t world, int32_t body, const float *linear, const float *angular);
/// 浮力（Jolt 的 ApplyBuoyancyImpulse）：水面是过 surface_position（世界坐标，双精度）、法线 surface_normal 的平面，按物体的形状
/// 算出水下的体积和浮心，加浮力和水的阻力（linear_drag、angular_drag），水按 fluid_velocity（米每秒）流；buoyancy 是浮力系数
/// （1 是中性，大于 1 浮起来），delta_time 是这一步的时长。每个物理步之前调。运动学物体忽略；*submerged 写 1（有部分在水下）或 0，
/// 不要时传 null。
SN_API int32_t sn_physics_body_apply_buoyancy(int32_t world, int32_t body, const double *surface_position, const float *surface_normal, float buoyancy, float linear_drag, float angular_drag, const float *fluid_velocity, float delta_time, int32_t *submerged);
/// 受不受重力（Stride 的 Gravity）。
SN_API int32_t sn_physics_body_set_gravity(int32_t world, int32_t body, int32_t enabled);
/// 一直醒着（SN_BODY_NEVER_SLEEP，Stride 的 SleepThreshold 为负）；打开时顺便唤醒。
SN_API int32_t sn_physics_body_set_never_sleep(int32_t world, int32_t body, int32_t never_sleep);

// ---------------------------------------------------------------------------------------------
// 约束（Stride.BepuPhysics 的约束组件，语义照 Bepu；Bepu 的约束都是软的，这里除了伺服和距离限制都是刚性的）
// ---------------------------------------------------------------------------------------------

enum
{
	SN_CONSTRAINT_BALL_SOCKET = 0,			///< offset_a、offset_b 两点重合（Jolt 的 PointConstraint）
	SN_CONSTRAINT_HINGE = 1,				///< 两点重合，axis_a、axis_b 两条铰链轴重合（HingeConstraint）
	SN_CONSTRAINT_ANGULAR_AXIS_GEAR_MOTOR = 2,	///< B 绕 A 的 axis_a 的角速度 = scale · A 绕它的角速度（GearConstraint）
	SN_CONSTRAINT_POINT_ON_LINE = 3,		///< B 的点（offset_b）在 A 上过 offset_a、方向 axis_a 的直线上，不限制转动
	SN_CONSTRAINT_LINEAR_AXIS_LIMIT = 4,	///< 两点沿 A 的 axis_a 的距离在 [minimum, maximum] 里
	SN_CONSTRAINT_SWING_LIMIT = 5,			///< axis_a、axis_b 的夹角不超过 maximum（弧度）
	SN_CONSTRAINT_TWIST_LIMIT = 6,			///< 绕 basis_a、basis_b 的 Z 轴的扭转（从 X 量向 Y）在 [minimum, maximum] 里
	SN_CONSTRAINT_WELD = 7,					///< B 在 A 的 offset_a 处、相对 A 的朝向是 basis_a（FixedConstraint）
	// 单体约束：只有 body_a，另一边是世界（body_b 忽略）。目标在世界空间
	SN_CONSTRAINT_ONE_BODY_LINEAR_SERVO = 8,	///< A 的 offset_a 点被弹簧（spring）拉向世界坐标 target，力不超过 max_force
	SN_CONSTRAINT_ONE_BODY_ANGULAR_SERVO = 9,	///< A 的朝向被弹簧拉向世界朝向 target（xyzw），力矩不超过 max_force
	SN_CONSTRAINT_ONE_BODY_ANGULAR_MOTOR = 10,	///< A 的角速度被拉向世界角速度 target，力矩不超过 max_force
	SN_CONSTRAINT_DISTANCE_LIMIT = 11,		///< offset_a、offset_b 两点的距离在 [minimum, maximum] 里，超出时用弹簧（spring）拉回（DistanceConstraint）
	SN_CONSTRAINT_BALL_SOCKET_MOTOR = 12,	///< B 的 offset_b 点相对 A 的速度被拉向 target（A 的本地空间），力不超过 max_force
};

/// 一个约束；向量和四元数都在各自物体的本地空间，四元数是 xyzw。不用的字段忽略。
typedef struct sn_constraint
{
	uint32_t type;					///< SN_CONSTRAINT_*
	int32_t body_a;
	int32_t body_b;
	float offset_a[3];
	float offset_b[3];
	float axis_a[3];
	float axis_b[3];
	float basis_a[4];
	float basis_b[4];
	float minimum;
	float maximum;
	float scale;
	float target[4];				///< 伺服和马达的目标，见各类型
	float spring[2];				///< 频率（Hz）、阻尼比；频率 <= 0 时是刚性的
	float max_force;				///< 伺服和马达的最大力（矩）；<= 0 时不限
} sn_constraint;

/// 加一个约束（两个物体都会唤醒），返回约束句柄（> 0）。
SN_API int32_t sn_physics_add_constraint(int32_t world, const sn_constraint *constraint);
/// 移除约束；移除过的忽略。物体被移除时连着它的约束一起移除。
SN_API int32_t sn_physics_remove_constraint(int32_t world, int32_t constraint);

// ---------------------------------------------------------------------------------------------
// 轮式车（Stride 没有；Jolt 的 VehicleConstraint 加 WheeledVehicleController）：车身是一个动态物体，轮子不是物体，
// 每步沿悬挂往下投一个圆柱找地面，悬挂、轮胎摩擦、发动机、自动变速箱、差速器、防倾杆都在 Jolt 里算。
// ---------------------------------------------------------------------------------------------

enum
{
	SN_VEHICLE_DRIVE_FRONT = 0,	///< 第一对轮子（前轴）驱动
	SN_VEHICLE_DRIVE_REAR = 1,	///< 最后一对轮子（后轴）驱动
	SN_VEHICLE_DRIVE_ALL = 2,	///< 每一对都驱动，发动机的扭矩平分
};

/// 一个轮子；位置、方向都在车身的本地空间。轮子按轴成对给：0、1 是第一根轴的左、右，2、3 是第二根轴的，依此类推。
typedef struct sn_vehicle_wheel
{
	float position[3];			///< 悬挂的上端（悬挂伸到最长时轮心在它下面 suspension_max）
	float radius;
	float width;
	float suspension_min;		///< 悬挂压到最短的长度（米）
	float suspension_max;		///< 悬挂伸到最长的长度（米）
	float suspension_frequency;	///< 悬挂弹簧的频率（Hz），1～2 是轿车
	float suspension_damping;	///< 阻尼比，0.5 左右
	float max_steer_angle;		///< 最大转向角（弧度），0 是不转向的轮子
	float max_brake_torque;		///< 刹车（牛·米）
	float max_hand_brake_torque;	///< 手刹（牛·米），一般只给后轮
} sn_vehicle_wheel;

/// 车的整体设置；方向在车身的本地空间。
typedef struct sn_vehicle
{
	int32_t body;				///< 车身（动态物体）
	uint32_t drive;				///< SN_VEHICLE_DRIVE_*
	float forward[3];			///< 车头朝哪（单位向量）；上方是 +Y
	float max_engine_torque;	///< 发动机最大扭矩（牛·米），500 是普通轿车
	float max_rpm;				///< 最高转速
	float max_pitch_roll_angle;	///< 车身倾斜不超过这个角度（弧度），防翻车；>= π 是不限
	float anti_roll_stiffness;	///< 每根轴的防倾杆（牛/米），0 是没有
} sn_vehicle;

/// 加一辆车，返回车的句柄（> 0）。车身被移除时车一起移除。轮子至少 2 个、个数是偶数。
SN_API int32_t sn_physics_add_vehicle(int32_t world, const sn_vehicle *vehicle, const sn_vehicle_wheel *wheels, uint32_t wheel_count);
/// 移除车（车身留着）；移除过的忽略。
SN_API int32_t sn_physics_remove_vehicle(int32_t world, int32_t vehicle);
/// 驾驶的输入：forward -1..1（油门，负的是倒车）、right -1..1（转向）、brake 0..1、hand_brake 0..1。有输入时唤醒车身。
SN_API int32_t sn_physics_vehicle_input(int32_t world, int32_t vehicle, float forward, float right, float brake, float hand_brake);
/// 车的状态：out[0] 发动机转速、out[1] 挡位（负的是倒挡）、out[2] 沿车头方向的速度（米/秒）、out[3] 着地的轮子数；
/// 之后每个轮子 8 个数：相对车身的位置 xyz、旋转 xyzw（轮子模型的轮轴沿本地 X）、悬挂现在的长度。capacity 是 out 能放几个数。
SN_API int32_t sn_physics_vehicle_state(int32_t world, int32_t vehicle, float *out, uint32_t capacity);

// ---------------------------------------------------------------------------------------------
// 布娃娃（Stride 没有；Jolt 的 Ragdoll）：骨架里选一部分关节，每个一个动态物体（胶囊沿骨头），父子之间是摆动加扭转的
// 约束，相邻的部分互不碰撞。物体的本地坐标就是关节的坐标；姿势和动画用同一种矩阵（关节到模型、列主序的 4x4）。
// ---------------------------------------------------------------------------------------------

/// 布娃娃的一部分（一个关节）。部分按父在前排，parent 是部分的下标（-1 是根，只能有一个）。
typedef struct sn_ragdoll_part
{
	int32_t joint;				///< 骨架里的关节下标（模型矩阵里的第几个）
	int32_t parent;				///< 父部分的下标；-1 是根
	float bone_end[3];			///< 骨头的另一头，在这个关节的本地空间：胶囊从关节原点到这里
	float radius;				///< 胶囊的半径
	float mass;					///< 千克
	float swing;				///< 和父部分之间摆动的最大角（弧度，圆锥的半角）
	float twist;				///< 绕骨头扭转的最大角（弧度，正负各这么多）
} sn_ragdoll_part;

/// 按骨架现在的姿势建布娃娃：model_matrices 是全部 joint_count 个关节的关节到模型的矩阵，model_to_world 是模型到世界的
/// 矩阵（双精度、列主序），layer 是碰撞层。返回句柄（> 0）。
SN_API int32_t sn_physics_add_ragdoll(int32_t world, const sn_ragdoll_part *parts, uint32_t part_count, const float *model_matrices, uint32_t joint_count, const double *model_to_world, uint32_t layer);
/// 移除布娃娃（它的物体一起移除）；移除过的忽略。
SN_API int32_t sn_physics_remove_ragdoll(int32_t world, int32_t ragdoll);
/// 根部分的世界位姿：position（双精度 xyz）、rotation（xyzw）。
SN_API int32_t sn_physics_ragdoll_root(int32_t world, int32_t ragdoll, double *position, float *rotation);
/// 物理的姿势写回骨架：model_to_world 是模型现在的位姿，model_matrices（joint_count 个，进来是上一次的姿势）里布娃娃的关节
/// 换成物体的位姿，别的关节跟着父关节（保持和父关节原来的相对位姿）；parents 是骨架每个关节的父关节（-1 是根，父在前）。
SN_API int32_t sn_physics_ragdoll_pose(int32_t world, int32_t ragdoll, const double *model_to_world, float *model_matrices, const int32_t *parents, uint32_t joint_count);
/// 给一部分加冲量（世界空间，牛·秒），比如被撞的那一下；part 是 -1 时是全身的冲量，按质量分给每一部分（全身的速度变化是 冲量 / 总质量）。
SN_API int32_t sn_physics_ragdoll_impulse(int32_t world, int32_t ragdoll, int32_t part, const float *impulse);

// ---------------------------------------------------------------------------------------------
// 接触事件（Stride 的 IContactHandler）
// ---------------------------------------------------------------------------------------------

enum
{
	SN_CONTACT_STARTED = 1,			///< 两个物体开始接触（OnStartedTouching、Trigger 的 OnEnter）
	SN_CONTACT_STOPPED = 2,			///< 不再接触（OnStoppedTouching、OnLeave）
	SN_CONTACT_TOUCHING = 3,		///< 这一步正在接触（OnTouching），depth 是这一步这对物体最深的穿透深度
};

/// 一个事件；body_a 是打开了 SN_BODY_CONTACT_EVENTS 的那个（两个都打开时按编号小的在前，只报一次）。
typedef struct sn_contact_event
{
	int32_t type;					///< SN_CONTACT_*
	int32_t body_a;
	int32_t body_b;
	float depth;					///< SN_CONTACT_TOUCHING 的穿透深度（米，正数是陷进去）；别的事件是 0
} sn_contact_event;

/// 打开或关掉某个物体的接触事件：0 关掉，1 报开始和结束（SN_BODY_CONTACT_EVENTS），2 另外每步报正在接触的深度
/// （SN_BODY_TOUCHING_EVENTS）。
SN_API int32_t sn_physics_body_set_contact_events(int32_t world, int32_t body, int32_t enabled);
/// 设成或取消传感器（SN_BODY_SENSOR）。
SN_API int32_t sn_physics_body_set_sensor(int32_t world, int32_t body, int32_t sensor);
/// 上一次 sn_physics_step 里产生的事件：开始和结束按发生的顺序，正在接触的在最后（每对物体一个）；地址在下一次步进前有效。
/// 复合形状的多对子形状按一对物体算。
SN_API const sn_contact_event *sn_physics_contact_events(int32_t world, uint32_t *count);

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
	double point[3];				///< 世界位置（双精度）
	float distance;					///< 沿方向的距离
	float normal[3];
	int32_t body;					///< 命中的物体编号
} sn_hit;

/// 射线（Stride 的 RayCast）：direction 不用归一化，max_distance 是米。mask 的第 n 位为 1 表示层 n 的物体参与。
/// 返回 1 命中（写 hit），0 没有，负数是错误。
SN_API int32_t sn_physics_ray_cast(int32_t world, const double *origin, const float *direction, float max_distance, uint32_t mask, sn_hit *hit);
/// 穿透的射线（Stride 的 RayCastPenetrating）：射线穿过的每个物体各一个命中（取进入它的那一处），从近到远写进 out_hits，
/// 最多 capacity 个；返回命中的物体总数（可能比 capacity 多），负数是错误。参数同 sn_physics_ray_cast。
SN_API int32_t sn_physics_ray_cast_all(int32_t world, const double *origin, const float *direction, float max_distance, uint32_t mask, sn_hit *out_hits, int32_t capacity);
enum
{
	/// 球体扫掠忽略起点就和球重叠的物体（默认它们算距离 0 的命中，同 Bepu）。比如第三人称相机：贴着旁边的箱子时
	/// 不该把相机拉到最近；要防止相机钻进正后方的墙，配一条射线。
	SN_CAST_IGNORE_INITIAL_OVERLAP = 1,
};

/// 球体扫掠（Stride 的 SweepCast 加 Sphere），同 sn_physics_ray_cast；flags 是 SN_CAST_*。
SN_API int32_t sn_physics_sphere_cast(int32_t world, float radius, const double *origin, const float *direction, float max_distance, uint32_t mask, uint32_t flags, sn_hit *hit);

/// 形状的重叠查询（Stride 的 BepuSimulation.Overlap）：查询的世界位置是 origin（双精度），shape 的 position、rotation 是相对它的
/// 位姿（size、mesh 同 sn_shape，网格不缩放），mask 同射线。和它重叠的物体编号（去掉重复）写进 out_bodies，最多 capacity 个；返回重叠的物体总数（可能比 capacity 多），
/// 负数是错误。触发器也算（同 Bepu）。
SN_API int32_t sn_physics_overlap(int32_t world, const double *origin, const sn_shape *shape, uint32_t mask, int32_t *out_bodies, int32_t capacity);

/// 静态物体表面的三角形（导航网格的输入，同 Stride 的 RecastMeshSystem 收集静态碰撞体）：mask 的第 n 位为 1 表示层 n 的物体参与，
/// 传感器（触发器）不算。三角形按世界坐标连续写，每个 9 个 float，从外面看是逆时针（法线朝外）。各种形状（盒、球、网格、
/// 复合……）都按 Jolt 的三角化。*out_vertices 用 sn_alloc 分配，调用方 sn_free（没有三角形时是 null）。返回三角形个数。
SN_API int32_t sn_physics_static_triangles(int32_t world, uint32_t mask, float **out_vertices);
