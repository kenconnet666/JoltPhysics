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
/// 位置和世界矩阵是双精度（大世界几十公里外也不抖）；render_world 是给 GPU 和剔除的单精度那份，平移相对渲染原点
/// （sn_transforms_set_render_origin）。
typedef struct sn_transform_columns
{
	double *position;				///< 本地位置，每个槽位 xyz
	float *rotation;				///< 本地旋转，每个槽位一个单位四元数 xyzw
	float *scale;					///< 本地缩放，每个槽位 xyz
	const int32_t *parent;			///< 父槽位，0 表示根；只能用 sn_transforms_set_parent 改
	const double *world;			///< 世界矩阵，每个槽位 16 个 double，列主序（平移在 12..14），由 sn_transforms_update 写
	const float *render_world;		///< 渲染用世界矩阵，每个槽位 16 个 float，平移减了渲染原点；和 world 一起写
	uint32_t capacity;				///< 最大槽位号；各列都有 capacity + 1 项（含槽位 0）
} sn_transform_columns;

/// 创建变换存储，返回句柄。capacity 是一开始能放的槽位数；不够时分配会自动增长（翻倍，上限 2^31 - 2），增长以后各列换了新的内存，
/// 要重新用 sn_transforms_columns 取（sn_transforms_generation 每增长一次加 1，用来判断要不要重取）。
SN_API int32_t sn_transforms_create(uint32_t capacity);
SN_API void sn_transforms_destroy(int32_t transforms);
SN_API int32_t sn_transforms_columns(int32_t transforms, sn_transform_columns *out_columns);
/// 存储增长过几次（出错时是负的错误码）。
SN_API int32_t sn_transforms_generation(int32_t transforms);
/// 分配 count 个槽位写进 out_slots：位置 0、旋转单位、缩放 1、没有父节点。返回 count；容量不够时先增长，到上限还不够才什么都不分配。
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
/// 渲染原点：render_world 的平移减去它。变了时所有槽位的 render_world 重算、都算"变了"（GPU 上的全部重传一次），所以按格
/// 对齐、相机走出一格才换（渲染器的做法）。
SN_API int32_t sn_transforms_set_render_origin(int32_t transforms, double x, double y, double z);
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

/// 视锥剔除（在渲染空间里：平面相对渲染原点，用 render_world）：实例的变换槽位按批次连续排在 slots 里，batch_ends[b] 是第 b 批的结束下标；同一批（同一个网格）共用一个本地
/// 包围盒，bounds 每批 6 个 float（中心 xyz、半长 xyz）。包围盒用槽位的世界矩阵变换（中心按点变换，半长按矩阵分量的绝对值
/// 变换，同 Stride 的 BoundingBoxExt.Transform），半长为 0 的包围盒总是可见、不参与量距离。第 v 个视图的结果写进
/// out_slots 的第 v * slot_count 段：每批可见的槽位按原顺序紧凑地放在这批原来的起点，个数写进 out_counts[v * batch_count + b]。
/// 视图数不设上限。
SN_API int32_t sn_transforms_cull(int32_t transforms, const int32_t *slots, uint32_t slot_count, const uint32_t *batch_ends, const float *bounds, uint32_t batch_count, sn_cull_view *views, uint32_t view_count, int32_t *out_slots, uint32_t *out_counts);

// ---------------------------------------------------------------------------------------------
// 矢量地形（stride-ts 的 W2）：地形是分层的描述——程序化的底（噪声）、矢量的地物（多边形压平）、雕刻的笔画（参数），按需求值成
// 高度页。坐标是双精度（大世界），同样的描述在任何机器上求出逐位相同的高度（碰撞、导航、渲染用同一份）。
// ---------------------------------------------------------------------------------------------

/// 程序化的底：分形噪声（梯度噪声，每层转一个角度），可以混进山脊、域扭曲，按导数压低陡处的细节（近似侵蚀的样子）。
typedef struct sn_terrain_noise
{
	uint32_t seed;
	uint32_t octaves;				///< 层数（1..16）
	float scale;					///< 最大的起伏的波长（米）
	float height;					///< 起伏的幅度（米）
	float offset;					///< 整体抬高（米）
	float lacunarity;				///< 每层频率乘多少（常用 2）
	float gain;						///< 每层幅度乘多少（常用 0.5）
	float ridged;					///< 0..1：混进多少山脊（1 − |噪声|）
	float warp;						///< 域扭曲的幅度（米）：采样点先按低频噪声挪一挪，山形不那么规整
	float erosion;					///< ≥ 0：按累积的坡度压低细节的强度（Inigo Quilez 的导数 fBm），陡处平滑、谷里留细节
	// 冲沟（侵蚀的样子，逐点算、不模拟）：沿下坡方向的条纹一层层分叉，坡缓的地方淡成谷底、山顶
	float gully_strength;			///< 冲沟的深度（米）；0 不做
	float gully_scale;				///< 最粗一层冲沟的间距（米）
	uint32_t gully_octaves;			///< 层数（0..8），每层间距减半
	float gully_detail;				///< > 0：细的冲沟要多陡才出现（越小越只长在陡处）
	float gully_valley;				///< 谷底的高度（米）：往下冲沟淡成谷
	float gully_peak;				///< 山顶的高度（米）：往上淡成尖
} sn_terrain_noise;

enum
{
	SN_TERRAIN_FEATURE_FLATTEN = 0,	///< 多边形里压到 height（边外 falloff 米内过渡）
	SN_TERRAIN_FEATURE_PATH = 1,	///< 折线（道路、河道）两侧 width / 2 米内压到折线的高度加 height（每个点的高度在 point_heights 里），再往外 falloff 米内过渡
};

enum
{
	SN_TERRAIN_PATH_CONFORM = 0,	///< 高了削、低了填
	SN_TERRAIN_PATH_CARVE = 1,		///< 只削（路堑、河道）
	SN_TERRAIN_PATH_FILL = 2,		///< 只填（路堤）
};

/// 矢量的地物：点在 points 表里（每个 x、z 两个 double），first_point 起 point_count 个。
typedef struct sn_terrain_feature
{
	uint32_t type;					///< SN_TERRAIN_FEATURE_*
	uint32_t first_point;
	uint32_t point_count;
	float height;					///< 目标高度（米）；折线是加在每个点的高度上的偏移
	float falloff;					///< 过渡的宽度（米）
	float strength;					///< 0..1
	float width;					///< 折线：压平的宽度（米，以折线为中心）
	uint32_t mode;					///< 折线：SN_TERRAIN_PATH_*
} sn_terrain_feature;

enum
{
	SN_TERRAIN_STROKE_RAISE = 0,	///< 抬高 strength 米（中心，往外按平滑的衰减）
	SN_TERRAIN_STROKE_LOWER = 1,	///< 压低
	SN_TERRAIN_STROKE_FLATTEN = 2,	///< 压向 height
	SN_TERRAIN_STROKE_SMOOTH = 3,	///< 磨平：靠向少几层细节的底
};

/// 雕刻的一笔（存参数，不存采样）：按顺序作用在底和地物上。
typedef struct sn_terrain_stroke
{
	double x;
	double z;
	uint32_t op;					///< SN_TERRAIN_STROKE_*
	float radius;					///< 米
	float strength;					///< 抬高、压低是米；压平、磨平是 0..1
	float height;					///< 压平的目标高度（米）
} sn_terrain_stroke;

/// 按描述建一个地形求值器（拷一份描述），返回句柄。
/// point_heights：每个点的高度（和 points 一样多，只有折线用；没有折线时可以是空）。
SN_API int32_t sn_terrain_create(const sn_terrain_noise *noise, const sn_terrain_feature *features, uint32_t feature_count, const double *points, const float *point_heights, uint32_t point_count, const sn_terrain_stroke *strokes, uint32_t stroke_count);
SN_API void sn_terrain_destroy(int32_t terrain);
/// 一点的高度（同步，玩法、编辑器摆东西用）。
SN_API float sn_terrain_height(int32_t terrain, double x, double z);
/// 一页高度（异步任务）：从 (x0, z0) 起、间距 spacing 米的 count_x × count_z 个点，按行（z）排。输出 [0]：float 的高度。
/// 求值器在任务开始前拷一份，任务跑着时销毁地形也安全。
SN_API int32_t sn_terrain_heights(int32_t terrain, double x0, double z0, double spacing, uint32_t count_x, uint32_t count_z);

// ---------------------------------------------------------------------------------------------
// BVH（光追用的包围盒层次，stride-ts 路线图 R2）
// ---------------------------------------------------------------------------------------------

/// 节点的布局（输出 [0]，4 叉的宽 BVH）：每个 32 个 32 位，[0..3] 4 个子节点包围盒的最小 x、[4..7] 最小 y、[8..11] 最小 z、[12..15]
/// 最大 x、[16..19] 最大 y、[20..23] 最大 z（float），[24..27] 子节点：内部节点是子节点的下标、叶子是第一个图元在输出 [1] 里的
/// 下标（uint32），[28..31] 子节点的图元数（0 是内部节点）。空着的子节点盒子是反的（最小角 FLT_MAX、最大角 −FLT_MAX，射线测不中）。
/// 根是第 0 个。GPU 上直接当存储缓冲用，一次读 8 个 vec4 测 4 个盒子（stride-ts 的 rendering/raytracing）。

/// 三角网格的 BVH（异步任务）：按表面积启发式（SAH，每个轴 16 个桶）建二叉树、再收成 4 叉，叶子最多 4 个三角形。positions 是 vertex_count × 3 个
/// float，indices 是 triangle_count × 3 个顶点下标。max_triangles 不是 0 且三角形比它多时先简化：连通的小块（包围盒对角线不到
/// 整个网格的 5%）按中心分进 2.5% 对角线的格子，每格换成一个包围盒，大块原样留着（简化后仍可能多于 max_triangles）。光追只要大致
/// 的形状——货架连同几千件商品几十万个三角形，商品换成盒子 BVH 小几倍；sloppy 简化会把小东西整个合没，不用。
/// 输出 [0]：节点；输出 [1]：按叶子排好的三角形，每个 12 个 32 位——v0（3 float）、三角形下标（简化过的是简化后的，uint32）、
/// e1 = v1 − v0（3 float）、0、e2 = v2 − v0（3 float）、0（Möller–Trumbore 直接用）。结果 [0]：节点数；结果 [1]：用了几个三角形。
SN_API int32_t sn_bvh_build_triangles(const float *positions, uint32_t vertex_count, const uint32_t *indices, uint32_t triangle_count, uint32_t max_triangles);
/// 包围盒的 BVH（异步任务，实例的顶层 BVH 用）：boxes 是 count × 6 个 float（最小角、最大角），叶子最多 2 个。
/// 输出 [0]：节点；输出 [1]：按叶子排好的图元下标（uint32）。结果 [0]：节点数。
SN_API int32_t sn_bvh_build_boxes(const float *boxes, uint32_t count);

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
	SN_ANIM_PUSH_SNAPSHOT = 4,		///< pushes the pose saved by sn_anim_instance_snapshot (blending back from a ragdoll)
	SN_ANIM_BLEND_MASKED = 5,		///< like BLEND, each joint's factor multiplied by the mask's weight (animation = the mask): layers
	SN_ANIM_ADD_MASKED = 6,			///< like ADD with a mask (an additive layer on the upper body)
};

/// A mask: one weight (0..1) per skeleton joint, for the masked blends (upper-body layers).
SN_API int32_t sn_anim_mask_create(const float *weights, uint32_t joint_count);
SN_API void sn_anim_mask_destroy(int32_t mask);

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

/// Inverse kinematics applied after an instance's program, in order, each on the result of the previous
/// (ozz's IKTwoBoneJob and IKAimJob): the corrected local rotations are written back and the model-space
/// matrices of the chain and everything below it are recomputed. Targets and poles are in model space.
enum
{
	SN_ANIM_IK_TWO_BONE = 0,		///< joints: start (thigh), mid (knee), end (foot); the end reaches target
	SN_ANIM_IK_AIM = 1,				///< joints[0] turns so that its forward axis points at target (head look-at)
	SN_ANIM_IK_OFFSET = 2,			///< joints[0] moves by target (model space), its children with it (hips down for foot placement)
};

enum
{
	SN_ANIM_IK_RELATIVE = 1,		///< two-bone, aim: target is an offset from where the end joint (aim: the joint) is when this IK runs
};

typedef struct sn_anim_ik
{
	int32_t type;					///< SN_ANIM_IK_*
	int32_t joints[3];				///< two-bone: start, mid, end; aim: joints[0]
	float target[3];				///< model space
	float pole[3];					///< model space: two-bone, where the mid joint (knee, elbow) points; aim, the model's up
	float axis[3];					///< two-bone: the mid joint's hinge axis (its local space); aim: the joint's forward axis (local)
	float up[3];					///< aim: the joint's up axis (local)
	float offset[3];				///< aim: the aiming point's offset from the joint (local)
	float weight;					///< 0 no correction, 1 full
	float soften;					///< two-bone: 1 is no softening, below 1 slows down when the chain is nearly straight
	float twist;					///< radians around the target direction
	uint32_t flags;					///< SN_ANIM_IK_*
	uint32_t reserved;
} sn_anim_ik;

/// Saves the instance's current model-space matrices (the last evaluation, or what a ragdoll wrote) as a
/// local pose that SN_ANIM_PUSH_SNAPSHOT pushes. root_correction (16 floats, column-major, or null)
/// multiplies the root joints' model matrices first: when the model itself moved (stood up after a
/// ragdoll), it is new model-to-world inverse times old model-to-world, so the pose stays put in the world.
SN_API int32_t sn_anim_instance_snapshot(int32_t instance, const float *root_correction);

/// Sets the IK applied after every evaluation of this instance (copied; count 0 clears it). Returns the
/// count, or an error when a joint index is out of range.
SN_API int32_t sn_anim_instance_set_ik(int32_t instance, const sn_anim_ik *iks, uint32_t count);

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
