// 由 scripts/generate-exports.mjs 从 Include/StrideNative/*.h 生成，不要手改：改头文件后重新生成（pnpm run build 会做）。
// 指针、句柄、整数和 float 在 JS 里都是 number。

/** Emscripten 模块上的 C 函数（名字前加 `_`），参数名和注释同头文件。 */
export interface NativeExports {
	/**
	 * 16-byte aligned allocation in module memory; free with sn_free.
	 * C: Core.h
	 */
	_sn_alloc(size: number): number;
	/** C: Runtime.h */
	_sn_anim_animation_duration(animation: number): number;
	/** C: Runtime.h */
	_sn_anim_destroy_animation(animation: number): void;
	/** C: Runtime.h */
	_sn_anim_destroy_skeleton(skeleton: number): void;
	/**
	 * Runs every request's program (at most 8 poses deep, leaving one pose), converts to model space, in
	 * parallel; returns when all are done.
	 * C: Runtime.h
	 */
	_sn_anim_evaluate_batch(ops: number, requests: number, count: number): number;
	/**
	 * An instance owns the sampling contexts, the pose stack and the model-space matrices of one character.
	 * C: Runtime.h
	 */
	_sn_anim_instance_create(skeleton: number): number;
	/** C: Runtime.h */
	_sn_anim_instance_destroy(instance: number): void;
	/**
	 * joint_count column-major 4x4 float matrices (joint-to-model), valid while the instance lives.
	 * C: Runtime.h
	 */
	_sn_anim_instance_model_matrices(instance: number): number;
	/** C: Runtime.h */
	_sn_anim_load_animation(data: number, size: number): number;
	/**
	 * Loads ozz archives (as produced by stride-tools). Return handles.
	 * C: Runtime.h
	 */
	_sn_anim_load_skeleton(data: number, size: number): number;
	/** C: Runtime.h */
	_sn_anim_skeleton_joint_count(skeleton: number): number;
	/**
	 * Writes the skinning matrices of every request, in parallel.
	 * C: Runtime.h
	 */
	_sn_anim_skin_batch(requests: number, count: number): number;
	/**
	 * A skin binds a mesh to joints of a skeleton: joints are skeleton joint indices, inverse_binds one
	 * column-major 4x4 matrix per joint (mesh to joint space at bind time).
	 * C: Runtime.h
	 */
	_sn_anim_skin_create(joints: number, inverseBinds: number, jointCount: number): number;
	/** C: Runtime.h */
	_sn_anim_skin_destroy(skin: number): void;
	/** C: Runtime.h */
	_sn_anim_skin_joint_count(skin: number): number;
	/**
	 * Returns the agent index.
	 * C: Navigation.h
	 */
	_sn_crowd_add_agent(crowd: number, position3: number, radius: number, height: number, maxSpeed: number, maxAcceleration: number): number;
	/** C: Navigation.h */
	_sn_crowd_create(navmesh: number, maxAgents: number, maxAgentRadius: number): number;
	/** C: Navigation.h */
	_sn_crowd_destroy(crowd: number): void;
	/**
	 * The crowd's positions buffer: xyz per agent slot (NaN for empty slots), room for max_agents. It stays
	 * at this address until the crowd is destroyed; add_agent, remove_agent and update keep it current.
	 * C: Navigation.h
	 */
	_sn_crowd_positions(crowd: number): number;
	/** C: Navigation.h */
	_sn_crowd_remove_agent(crowd: number, agent: number): void;
	/** C: Navigation.h */
	_sn_crowd_request_move(crowd: number, agent: number, target3: number): number;
	/**
	 * Moves the agents, then writes their positions into the positions buffer.
	 * C: Navigation.h
	 */
	_sn_crowd_update(crowd: number, deltaTime: number): void;
	/**
	 * Adds one call that took `milliseconds` to a timing, e.g. "physics.step".
	 * C: Debug.h
	 */
	_sn_debug_add_timing(name: number, milliseconds: number): void;
	/**
	 * Queued log lines as a JSON array [{"level":n,"message":"..."}], removed from the queue. The string
	 * stays valid until the next call on the calling thread.
	 * C: Debug.h
	 */
	_sn_debug_drain_log(): number;
	/** C: Debug.h */
	_sn_debug_enabled(): number;
	/**
	 * Queues a log line for JS. While collection is off only SN_LOG_ERROR lines are queued, so an error
	 * before the debugger attaches is not lost. The queue is bounded (oldest lines are dropped).
	 * C: Debug.h
	 */
	_sn_debug_log(level: number, message: number): void;
	/**
	 * Clears the timings (counters keep their last values).
	 * C: Debug.h
	 */
	_sn_debug_reset_timings(): void;
	/**
	 * Sets a counter to its current value, e.g. "physics.bodies".
	 * C: Debug.h
	 */
	_sn_debug_set_counter(name: number, value: number): void;
	/** C: Debug.h */
	_sn_debug_set_enabled(enabled: number): void;
	/**
	 * JSON object {"counters":{name:value},"timings":{name:{"calls":n,"ms":total,"maxMs":m}},"memory":{...}}.
	 * The string stays valid until the next call on the calling thread.
	 * C: Debug.h
	 */
	_sn_debug_snapshot(): number;
	/** C: Core.h */
	_sn_free(ptr: number): void;
	/**
	 * Starts the shared job system. worker_threads < 0 uses (hardware threads - 1); 0 is rejected because
	 * tasks only run on worker threads. Call once.
	 * C: Core.h
	 */
	_sn_init(workerThreads: number): number;
	/** C: Texture.h */
	_sn_ktx2_get_info(data: number, size: number, outInfo: number): number;
	/**
	 * Transcodes every mip level in parallel. data must stay alive until the task finishes.
	 * Outputs: one buffer per level (level 0 first) for layer 0 / face 0.
	 * Results: [0] width, [1] height, [2] level count, [3] bytes per block (or per pixel for RGBA8).
	 * C: Texture.h
	 */
	_sn_ktx2_transcode(data: number, size: number, target: number): number;
	/**
	 * Message of the last failed call on the calling thread, or "" if none.
	 * C: Core.h
	 */
	_sn_last_error(): number;
	/**
	 * Decodes one glTF buffer view into dst (count * stride bytes).
	 * C: Runtime.h
	 */
	_sn_meshopt_decode(dst: number, count: number, stride: number, src: number, srcSize: number, mode: number, filter: number): number;
	/**
	 * Decodes many buffer views in parallel. The requests and buffers must stay alive until the task
	 * finishes. Returns a task; it fails if any request failed (see each request's result).
	 * C: Runtime.h
	 */
	_sn_meshopt_decode_batch(requests: number, count: number): number;
	/**
	 * Builds a tiled navmesh from a triangle soup, tiles in parallel. Inputs must stay alive until
	 * the task finishes. Result [0]: navmesh handle; [1]: tile count.
	 * C: Navigation.h
	 */
	_sn_nav_build(vertices: number, vertexCount: number, triangles: number, triangleCount: number, config: number): number;
	/**
	 * Fills a config with the Recast demo defaults for a human-sized agent.
	 * C: Navigation.h
	 */
	_sn_nav_default_config(outConfig: number): void;
	/** C: Navigation.h */
	_sn_nav_destroy(navmesh: number): void;
	/**
	 * Straight path between two points (corners written as xyz). Returns the corner count.
	 * C: Navigation.h
	 */
	_sn_nav_find_path(navmesh: number, start3: number, end3: number, searchExtents3: number, outPoints: number, maxPoints: number): number;
	/**
	 * Loads a navmesh serialized by sn_nav_serialize or the stride-tools bake. Returns a handle.
	 * C: Navigation.h
	 */
	_sn_nav_load(data: number, size: number): number;
	/**
	 * Serializes a navmesh. Result buffer is allocated with sn_alloc; the caller frees it.
	 * C: Navigation.h
	 */
	_sn_nav_serialize(navmesh: number, outData: number, outSize: number): number;
	/**
	 * sn_shutdown 时调用 fn(user)（后注册的先调用），插件在这里释放自己的对象。注册一直有效，跨多次 init/shutdown，
	 * 所以只注册一次（比如第一次用到时）。
	 * C: Plugin.h
	 */
	_sn_on_shutdown(fn: number, user: number): void;
	/** C: Plugin.h */
	_sn_parallel_for(count: number, fn: number, user: number): void;
	/**
	 * 一次加入 count 个物体（Jolt 成批插进宽相），每个物体的编号写进 out_bodies（可以是 null），返回第一个物体的编号。
	 * 先复用移除过的物体的编号，所以编号不一定连续；没有移除过的物体时按加入顺序递增。同时存在的物体不超过 max_bodies。
	 * C: Physics.h
	 */
	_sn_physics_add_bodies(world: number, bodies: number, count: number, shapes: number, shapeCount: number, characters: number, characterCount: number, outBodies: number): number;
	/**
	 * 加一个约束（两个物体都会唤醒），返回约束句柄（> 0）。
	 * C: Physics.h
	 */
	_sn_physics_add_constraint(world: number, constraint: number): number;
	/**
	 * 在质心加冲量（Stride 的 ApplyLinearImpulse、ApplyAngularImpulse）；传 null 的不加。运动学物体忽略。
	 * C: Physics.h
	 */
	_sn_physics_body_apply_impulse(world: number, body: number, linear: number, angular: number): number;
	/**
	 * 1 醒着，0 睡着，负数是错误。
	 * C: Physics.h
	 */
	_sn_physics_body_awake(world: number, body: number): number;
	/**
	 * 唤醒或让它睡下（Stride 的 Awake）。
	 * C: Physics.h
	 */
	_sn_physics_body_set_awake(world: number, body: number, awake: number): number;
	/**
	 * 打开或关掉某个物体的接触事件：0 关掉，1 报开始和结束（SN_BODY_CONTACT_EVENTS），2 另外每步报正在接触的深度
	 * （SN_BODY_TOUCHING_EVENTS）。
	 * C: Physics.h
	 */
	_sn_physics_body_set_contact_events(world: number, body: number, enabled: number): number;
	/**
	 * 受不受重力（Stride 的 Gravity）。
	 * C: Physics.h
	 */
	_sn_physics_body_set_gravity(world: number, body: number, enabled: number): number;
	/**
	 * 在动态和运动学之间切换（Stride 的 Kinematic）：只对按动态或运动学加入的物体有效。
	 * C: Physics.h
	 */
	_sn_physics_body_set_kinematic(world: number, body: number, kinematic: number): number;
	/**
	 * 一直醒着（SN_BODY_NEVER_SLEEP，Stride 的 SleepThreshold 为负）；打开时顺便唤醒。
	 * C: Physics.h
	 */
	_sn_physics_body_set_never_sleep(world: number, body: number, neverSleep: number): number;
	/**
	 * 只改朝向（Stride 的 SetTargetPose(Quaternion)、CharacterComponent.Orientation），并唤醒：位置和它的插值不动，所以可以每帧调
	 * （鼠标转视角带着角色转）。绑定了变换的立即写回旋转。角色之后的 MoveVector 按新的朝向换到世界。
	 * C: Physics.h
	 */
	_sn_physics_body_set_rotation(world: number, body: number, rotation: number): number;
	/**
	 * 设成或取消传感器（SN_BODY_SENSOR）。
	 * C: Physics.h
	 */
	_sn_physics_body_set_sensor(world: number, body: number, sensor: number): number;
	/**
	 * 设线速度和角速度；传 null 的不变。
	 * C: Physics.h
	 */
	_sn_physics_body_set_velocity(world: number, body: number, linear: number, angular: number): number;
	/**
	 * 瞬移到世界位姿（Stride 的 Teleport），并唤醒；绑定了变换的立即写回槽位，插值也从这里重新开始。角色也可以（转身改朝向，
	 * Stride 的 CharacterComponent.Orientation），之后的 MoveVector 按新的朝向换到世界。
	 * C: Physics.h
	 */
	_sn_physics_body_teleport(world: number, body: number, position: number, rotation: number): number;
	/**
	 * 读线速度和角速度（世界空间，各 xyz）；不要的传 null。角色也可以读（它的速度由移动目标每步设，不能直接设）。
	 * C: Physics.h
	 */
	_sn_physics_body_velocity(world: number, body: number, linear: number, angular: number): number;
	/**
	 * 角色是否站在可以站立的地面上（Stride 的 IsGrounded）：1 是，0 否，负数是错误。
	 * C: Physics.h
	 */
	_sn_physics_character_grounded(world: number, body: number): number;
	/**
	 * 设下一步的移动：target 是世界 X、Z 方向的目标速度（Stride 的 MoveVector · Speed），jump 不为 0 时试着起跳
	 * （同 TryJump：只在站稳时生效，下一步用掉）。
	 * C: Physics.h
	 */
	_sn_physics_character_move(world: number, body: number, targetX: number, targetZ: number, jump: number): number;
	/**
	 * 上一次 sn_physics_step 里产生的事件：开始和结束按发生的顺序，正在接触的在最后（每对物体一个）；地址在下一次步进前有效。
	 * 复合形状的多对子形状按一对物体算。
	 * C: Physics.h
	 */
	_sn_physics_contact_events(world: number, count: number): number;
	/**
	 * 建一个在共享任务系统上步进的世界，返回句柄。transforms 是物体可以绑定的变换存储（0 表示不用），它要比世界
	 * 活得久，绑定的槽位在物体存在期间不能释放。settings 为空时用 Stride 的默认值（重力 -9.8、阻尼 0.05、全部层相互碰撞）。
	 * C: Physics.h
	 */
	_sn_physics_create(maxBodies: number, transforms: number, settings: number): number;
	/**
	 * 凸包形状（Jolt 由点算出凸包），世界内的物体共用；和网格共用句柄空间。positions 是 xyz。返回句柄。
	 * C: Physics.h
	 */
	_sn_physics_create_hull(world: number, positions: number, pointCount: number): number;
	/**
	 * 三角网格形状，世界内的物体共用。positions 是 xyz，indices 每三个一个三角形（逆时针为正面）。返回网格句柄。
	 * C: Physics.h
	 */
	_sn_physics_create_mesh(world: number, positions: number, vertexCount: number, indices: number, indexCount: number): number;
	/** C: Physics.h */
	_sn_physics_destroy(world: number): void;
	/**
	 * 插值的物体写回 alpha（0 是上一步，1 是这一步）处的位姿。
	 * C: Physics.h
	 */
	_sn_physics_interpolate(world: number, alpha: number): number;
	/**
	 * 形状的重叠查询（Stride 的 BepuSimulation.Overlap）：shape 的 position、rotation 是查询的世界位姿（size、mesh 同 sn_shape，网格
	 * 不缩放），mask 同射线。和它重叠的物体编号（去掉重复）写进 out_bodies，最多 capacity 个；返回重叠的物体总数（可能比 capacity 多），
	 * 负数是错误。触发器也算（同 Bepu）。
	 * C: Physics.h
	 */
	_sn_physics_overlap(world: number, shape: number, mask: number, outBodies: number, capacity: number): number;
	/**
	 * 每个物体的位置 xyz，按编号排，能放 max_bodies 个；地址在世界销毁前不变，add_bodies 和 step 原地改写。
	 * C: Physics.h
	 */
	_sn_physics_positions(world: number): number;
	/**
	 * 射线（Stride 的 RayCast）：direction 不用归一化，max_distance 是米。mask 的第 n 位为 1 表示层 n 的物体参与。
	 * 返回 1 命中（写 hit），0 没有，负数是错误。
	 * C: Physics.h
	 */
	_sn_physics_ray_cast(world: number, origin: number, direction: number, maxDistance: number, mask: number, hit: number): number;
	/**
	 * 穿透的射线（Stride 的 RayCastPenetrating）：射线穿过的每个物体各一个命中（取进入它的那一处），从近到远写进 out_hits，
	 * 最多 capacity 个；返回命中的物体总数（可能比 capacity 多），负数是错误。参数同 sn_physics_ray_cast。
	 * C: Physics.h
	 */
	_sn_physics_ray_cast_all(world: number, origin: number, direction: number, maxDistance: number, mask: number, outHits: number, capacity: number): number;
	/**
	 * 移除物体（实体销毁时）：从世界里拿走并释放，编号留给以后加的物体复用；和它有关的接触不再报告。移除过的忽略。
	 * C: Physics.h
	 */
	_sn_physics_remove_bodies(world: number, bodies: number, count: number): number;
	/**
	 * 移除约束；移除过的忽略。物体被移除时连着它的约束一起移除。
	 * C: Physics.h
	 */
	_sn_physics_remove_constraint(world: number, constraint: number): number;
	/**
	 * 运行中改重力（Stride 的 BepuSimulation.PoseGravity）；睡着的物体要被碰到或唤醒才受新的重力。
	 * C: Physics.h
	 */
	_sn_physics_set_gravity(world: number, gravity: number): number;
	/**
	 * 球体扫掠（Stride 的 SweepCast 加 Sphere），同 sn_physics_ray_cast；flags 是 SN_CAST_*。
	 * C: Physics.h
	 */
	_sn_physics_sphere_cast(world: number, radius: number, origin: number, direction: number, maxDistance: number, mask: number, flags: number, hit: number): number;
	/**
	 * 静态物体表面的三角形（导航网格的输入，同 Stride 的 RecastMeshSystem 收集静态碰撞体）：mask 的第 n 位为 1 表示层 n 的物体参与，
	 * 传感器（触发器）不算。三角形按世界坐标连续写，每个 9 个 float，从外面看是逆时针（法线朝外）。各种形状（盒、球、网格、
	 * 复合……）都按 Jolt 的三角化。*out_vertices 用 sn_alloc 分配，调用方 sn_free（没有三角形时是 null）。返回三角形个数。
	 * C: Physics.h
	 */
	_sn_physics_static_triangles(world: number, mask: number, outVertices: number): number;
	/**
	 * 步进一个固定步长：先跑角色逻辑，再 Jolt 步进，再更新角色的支撑状态，最后写回位姿（插值的物体记下前后两步）。
	 * C: Physics.h
	 */
	_sn_physics_step(world: number, deltaTime: number, collisionSteps: number): number;
	/**
	 * 正在运行的核心的插件 ABI 版本。
	 * C: Plugin.h
	 */
	_sn_plugin_abi_version(): number;
	/**
	 * 启动任务，返回任务 id（> 0）。name 要一直有效（用字符串字面量），性能分析里显示它。free_user 不为空时，
	 * fn 返回后用 user 调用它。
	 * C: Plugin.h
	 */
	_sn_run_task(name: number, fn: number, user: number, freeUser: number): number;
	/**
	 * 记下当前线程的错误消息（JS 通过 sn_last_error 读），返回 code。
	 * C: Plugin.h
	 */
	_sn_set_last_error(code: number, message: number): number;
	/** C: Core.h */
	_sn_shutdown(): void;
	/**
	 * 加一个归任务所有的输出缓冲（sn_task_release 时释放），返回它的地址，由 fn 填写。
	 * C: Plugin.h
	 */
	_sn_task_add_output(task: number, size: number): number;
	/** C: Core.h */
	_sn_task_error(task: number): number;
	/**
	 * 标记任务失败；第一条消息有效。
	 * C: Plugin.h
	 */
	_sn_task_fail(task: number, message: number): void;
	/**
	 * Output buffers produced by the task. They stay valid until sn_task_release.
	 * C: Core.h
	 */
	_sn_task_output_count(task: number): number;
	/** C: Core.h */
	_sn_task_output_data(task: number, index: number): number;
	/** C: Core.h */
	_sn_task_output_size(task: number, index: number): number;
	/**
	 * Frees the task and its outputs. A running task is released as soon as it finishes.
	 * C: Core.h
	 */
	_sn_task_release(task: number): void;
	/**
	 * Small integer results (handles, counts, dimensions); meaning depends on the operation.
	 * index is 0..SN_TASK_RESULT_COUNT-1; other indices read 0.
	 * C: Core.h
	 */
	_sn_task_result(task: number, index: number): number;
	/**
	 * 小整数结果，index 是 0..SN_TASK_RESULT_COUNT-1（8 个），超出的忽略。
	 * C: Plugin.h
	 */
	_sn_task_set_result(task: number, index: number, value: number): void;
	/** C: Core.h */
	_sn_task_status(task: number): number;
	/** C: Core.h */
	_sn_task_status_address(task: number): number;
	/**
	 * 分配 count 个槽位写进 out_slots：位置 0、旋转单位、缩放 1、没有父节点。返回 count；容量不够时什么都不分配。
	 * C: Runtime.h
	 */
	_sn_transforms_alloc(transforms: number, outSlots: number, count: number): number;
	/** C: Runtime.h */
	_sn_transforms_columns(transforms: number, outColumns: number): number;
	/**
	 * 创建能放 capacity 个槽位的变换存储，返回句柄。
	 * C: Runtime.h
	 */
	_sn_transforms_create(capacity: number): number;
	/**
	 * 视锥剔除：实例的变换槽位按批次连续排在 slots 里，batch_ends[b] 是第 b 批的结束下标；同一批（同一个网格）共用一个本地
	 * 包围盒，bounds 每批 6 个 float（中心 xyz、半长 xyz）。包围盒用槽位的世界矩阵变换（中心按点变换，半长按矩阵分量的绝对值
	 * 变换，同 Stride 的 BoundingBoxExt.Transform），半长为 0 的包围盒总是可见、不参与量距离。第 v 个视图的结果写进
	 * out_slots 的第 v * slot_count 段：每批可见的槽位按原顺序紧凑地放在这批原来的起点，个数写进 out_counts[v * batch_count + b]。
	 * C: Runtime.h
	 */
	_sn_transforms_cull(transforms: number, slots: number, slotCount: number, batchEnds: number, bounds: number, batchCount: number, views: number, viewCount: number, outSlots: number, outCounts: number): number;
	/** C: Runtime.h */
	_sn_transforms_destroy(transforms: number): void;
	/**
	 * 释放这些槽位。还有子节点留着的槽位不能释放（一起释放的子节点不算），这时什么都不释放。
	 * C: Runtime.h
	 */
	_sn_transforms_free(transforms: number, slots: number, count: number): number;
	/**
	 * 链接（Stride 的 TransformLink，比如 ModelNodeLinkComponent 挂到骨骼节点）：slot 的世界矩阵改成
	 * link_slot.world * matrix * T * R * S，不再看父节点（父子关系不变）。matrix 是列主序 4x4，拷进存储；每帧要变时再调一次。
	 * link_slot 为 0 时去掉链接。link_slot 不能依赖 slot（会成环）。槽位释放时，它上面的和指向它的链接一起去掉。
	 * C: Runtime.h
	 */
	_sn_transforms_set_link(transforms: number, slot: number, linkSlot: number, matrix: number): number;
	/**
	 * 把 slot 挂到 parent 下（parent 为 0 表示变成根）。不能形成环。
	 * C: Runtime.h
	 */
	_sn_transforms_set_parent(transforms: number, slot: number, parent: number): number;
	/**
	 * 发出去过的最大槽位号（释放了也不降）：用到的槽位都在 1..它 之内，上传世界矩阵时只传这一段。
	 * C: Runtime.h
	 */
	_sn_transforms_slot_limit(transforms: number): number;
	/**
	 * 按父节点（有链接时是链接的槽位）在前的顺序算出所有槽位的世界矩阵：world = parent.world * T * R * S。
	 * C: Runtime.h
	 */
	_sn_transforms_update(transforms: number): number;
	/**
	 * 只算 slot 和它的祖先的世界矩阵（Stride 的 TransformComponent.UpdateWorldMatrix）：脚本在这一帧的 sn_transforms_update
	 * 之前要用世界位姿时调用。
	 * C: Runtime.h
	 */
	_sn_transforms_update_slot(transforms: number, slot: number): number;
	/** C: Core.h */
	_sn_worker_thread_count(): number;
	/**
	 * Builds an ozz animation archive. For each joint, key_counts holds (translation, rotation,
	 * scale) counts; keys are packed per joint in that order: translation = time,x,y,z;
	 * rotation = time,x,y,z,w; scale = time,x,y,z. Times are in seconds.
	 * C: Tools.h
	 */
	_st_anim_build_animation(duration: number, jointCount: number, keyCounts: number, keys: number, outData: number, outSize: number): number;
	/**
	 * Builds an ozz skeleton archive. names: joint_count NUL-terminated strings back to back (may
	 * be null). Output buffer is allocated with sn_alloc; the caller frees it.
	 * C: Tools.h
	 */
	_st_anim_build_skeleton(joints: number, jointCount: number, names: number, outData: number, outSize: number): number;
	/**
	 * Encodes an RGBA8 image into a KTX2 file. rgba must stay alive until the task finishes.
	 * quality 0-100, effort 0-10. Output [0]: the .ktx2 file.
	 * C: Tools.h
	 */
	_st_basis_encode(rgba: number, width: number, height: number, mode: number, flags: number, quality: number, effort: number): number;
	/**
	 * Convex decomposition (V-HACD) for physics. Inputs must stay alive until the task finishes.
	 * Outputs: [0] int32 pairs (vertex count, triangle count) per hull, [1] float3 vertices of all
	 * hulls, [2] uint32 triangle indices, local to their hull. Result [0]: hull count.
	 * C: Tools.h
	 */
	_st_convex_decompose(positions: number, vertexCount: number, indices: number, indexCount: number, maxHulls: number, voxelResolution: number, maxVerticesPerHull: number): number;
	/**
	 * Prefilters a radiance cubemap for GGX environment lighting like Stride's RadiancePrefilteringGGX:
	 * level 0 of the out_size cube is the source resampled, level i is filtered with roughness i / (levels - 1)
	 * using samples GGX importance samples. faces is level 0 of a size x size cube (mipmaps are generated);
	 * both sizes must be powers of two. faces must stay alive until the task finishes.
	 * Output [0]: every level from out_size down to 1, each 6 faces, concatenated.
	 * C: Tools.h
	 */
	_st_cubemap_prefilter_ggx(faces: number, size: number, outSize: number, samples: number): number;
	/**
	 * Projects a radiance cubemap onto order-3 spherical harmonics with the Lambertian cosine lobe, like Stride's
	 * LambertianPrefilteringSH; the coefficients are already multiplied by the basis constants, so evaluating
	 * them only needs the polynomials. faces must stay alive until the task finishes.
	 * Output [0]: 9 RGB coefficients (27 floats).
	 * C: Tools.h
	 */
	_st_cubemap_sh_lambert(faces: number, size: number): number;
	/**
	 * Every codepoint the font maps (U+0000 to U+10FFFF) and the legacy kern table. font must stay alive until the task
	 * finishes. Outputs: [0] uint32 codepoints ascending, [1] uint32 glyph index per codepoint, [2] kern pairs as int32
	 * (glyph1, glyph2, advance in font units), sorted by glyph1 then glyph2.
	 * C: Tools.h
	 */
	_st_font_codepoints(font: number, fontSize: number, index: number): number;
	/**
	 * Describes the font at index of a font file or collection (synchronous).
	 * C: Tools.h
	 */
	_st_font_describe(font: number, fontSize: number, index: number, outDescription: number): number;
	/**
	 * Multi-channel signed distance field glyphs (stb_truetype outlines, msdfgen), in parallel.
	 * font and codepoints must stay alive until the task finishes. Outputs: [0] st_glyph_metrics per
	 * codepoint, then one RGB8 bitmap per codepoint (empty for glyphs without outline).
	 * C: Tools.h
	 */
	_st_font_msdf(font: number, fontSize: number, codepoints: number, codepointCount: number, pixelSize: number, distanceRange: number): number;
	/**
	 * Glyph outlines as quadratic Bezier curves for the vector text renderer, in parallel. Cubic (CFF) segments are split
	 * into quadratics within tolerance font units; a line is a quadratic whose control point equals its end point.
	 * font and glyphs must stay alive until the task finishes. Outputs: [0] st_glyph_outline per glyph, [1] float curves
	 * (x1 y1 x2 y2 x3 y3, font units, y up), each glyph's contours closed.
	 * C: Tools.h
	 */
	_st_font_outlines(font: number, fontSize: number, index: number, glyphs: number, glyphCount: number, tolerance: number): number;
	/**
	 * MikkTSpace tangents, one xyzw per index (corner); weld afterwards if needed.
	 * C: Tools.h
	 */
	_st_generate_tangents(positions: number, normals: number, uvs: number, indices: number, indexCount: number, outTangents4: number): number;
	/**
	 * Decodes a PNG, JPEG, TGA or DDS file (the largest mip: BC1-BC5, BC7, 24/32-bit RGB) into RGBA8. data must stay alive until the task finishes.
	 * Result [0]: width, [1]: height, [2]: channels in the file. Output [0]: the pixels, rows top-down.
	 * C: Tools.h
	 */
	_st_image_decode(data: number, size: number): number;
	/**
	 * Resamples RGBA8 pixels (rows top-down) to out_width x out_height with a Lanczos3 filter on the stored values, edges clamped
	 * (Stride's TextureTool.Resize with Filter.Rescaling.Lanczos3). rgba must stay alive until the task finishes.
	 * Output [0]: the resampled pixels.
	 * C: Tools.h
	 */
	_st_image_resize(rgba: number, width: number, height: number, outWidth: number, outHeight: number): number;
	/** C: Tools.h */
	_st_meshopt_encode_index_bound(indexCount: number, vertexCount: number): number;
	/** C: Tools.h */
	_st_meshopt_encode_indices(dst: number, dstCapacity: number, indices: number, indexCount: number): number;
	/** C: Tools.h */
	_st_meshopt_encode_vertex_bound(vertexCount: number, vertexSize: number): number;
	/**
	 * Encode for SN_MESHOPT_MODE_ATTRIBUTES / SN_MESHOPT_MODE_TRIANGLES. Return the encoded size (0 on failure).
	 * C: Tools.h
	 */
	_st_meshopt_encode_vertices(dst: number, dstCapacity: number, vertices: number, vertexCount: number, vertexSize: number): number;
	/**
	 * Vertex cache plus overdraw optimization, in place.
	 * C: Tools.h
	 */
	_st_meshopt_optimize(indices: number, indexCount: number, positions: number, vertexCount: number, positionStride: number): number;
	/**
	 * Returns the new index count; out_error receives the relative error reached.
	 * C: Tools.h
	 */
	_st_meshopt_simplify(outIndices: number, indices: number, indexCount: number, positions: number, vertexCount: number, positionStride: number, targetIndexCount: number, targetError: number, outError: number): number;
	/**
	 * Same tiled parallel build as sn_nav_build, but outputs the serialized navmesh for
	 * sn_nav_load. Output [0]: navmesh data. Result [0]: tile count.
	 * C: Tools.h
	 */
	_st_nav_bake(vertices: number, vertexCount: number, triangles: number, triangleCount: number, config: number): number;
	/**
	 * Bakes an animation into linearly interpolated local-transform keys for every node, resampled at sample_rate
	 * keys per second, times starting at 0 (FBX: from the start of playback; glTF: from 0, ending at the last key).
	 * Returns a task; outputs: [0] key counts (uint32, translation, rotation, scale per node), [1] keys in
	 * st_anim_build_animation's layout, [2] the duration in seconds (one float), [3] morph weight key counts (uint32,
	 * one per target of every mesh with morph targets, in mesh then target order; 0 when the animation leaves the
	 * weight alone), [4] morph weight keys (time, weight). Nodes the animation leaves alone get one key of their rest transform.
	 * C: Tools.h
	 */
	_st_scene_animation_bake(scene: number, animation: number, sampleRate: number): number;
	/**
	 * Name of an animation (glTF animation, FBX animation stack), valid until the scene is destroyed; null for a bad index.
	 * C: Tools.h
	 */
	_st_scene_animation_name(scene: number, animation: number): number;
	/** C: Tools.h */
	_st_scene_destroy(scene: number): void;
	/**
	 * Parses a scene. data must stay alive until the task finishes. Result [0]: scene handle,
	 * [1]: mesh count, [2]: material count, [3]: node count, [4]: animation count.
	 * FBX's root node is named "RootNode" (like Stride's importer); nodes keep the file's order.
	 * C: Tools.h
	 */
	_st_scene_load(data: number, size: number, format: number): number;
	/**
	 * Name of a material (UTF-8), valid until the scene is destroyed; null for a bad index.
	 * C: Tools.h
	 */
	_st_scene_material_name(scene: number, material: number): number;
	/**
	 * Copies a mesh; any output may be null. positions/normals xyz, uvs xy, indices uint32.
	 * C: Tools.h
	 */
	_st_scene_mesh_copy(scene: number, mesh: number, positions: number, normals: number, uvs: number, indices: number): number;
	/** C: Tools.h */
	_st_scene_mesh_info(scene: number, mesh: number, outInfo: number): number;
	/**
	 * Deltas of a morph target, xyz per vertex in the mesh's (baked, model) space; any output may be null. normal_deltas
	 * stays untouched when the targets have no normals (st_mesh_info.morph_has_normals).
	 * C: Tools.h
	 */
	_st_scene_mesh_morph(scene: number, mesh: number, target: number, positionDeltas: number, normalDeltas: number): number;
	/**
	 * Name of a morph target of a mesh (UTF-8: glTF's extras.targetNames, FBX's blend channel), valid until the scene
	 * is destroyed; null for a bad index.
	 * C: Tools.h
	 */
	_st_scene_mesh_morph_name(scene: number, mesh: number, target: number): number;
	/**
	 * Skin of a skinned mesh (st_mesh_info.joint_count > 0); any output may be null.
	 * joint_nodes: joint_count node indices. inverse_binds: joint_count column-major 4x4 matrices taking the
	 * mesh's (baked, model-space) vertices to the joint's space at bind time, so a joint's skinning matrix is
	 * its current node-to-model matrix times its inverse bind. joints/weights: 4 per vertex, indices into
	 * joint_nodes sorted by decreasing weight, weights normalized (unused slots have weight 0).
	 * C: Tools.h
	 */
	_st_scene_mesh_skin(scene: number, mesh: number, jointNodes: number, inverseBinds: number, joints: number, weights: number): number;
	/** C: Tools.h */
	_st_scene_node_info(scene: number, node: number, outInfo: number): number;
	/**
	 * Name of a node (UTF-8), valid until the scene is destroyed; null for a bad index.
	 * C: Tools.h
	 */
	_st_scene_node_name(scene: number, node: number): number;
	/**
	 * Lightmap UV unwrap (xatlas). Inputs must stay alive until the task finishes.
	 * Outputs: [0] uint32 original-vertex index per new vertex, [1] float2 uv in [0,1] per new
	 * vertex, [2] uint32 new indices. Results: [0] atlas width, [1] atlas height, [2] new vertex count.
	 * C: Tools.h
	 */
	_st_uv_unwrap(positions: number, normals: number, vertexCount: number, indices: number, indexCount: number, texelsPerUnit: number): number;
}

/** Core.h、Debug.h 里的函数。 */
export type CoreExport =
	| 'sn_init'
	| 'sn_shutdown'
	| 'sn_worker_thread_count'
	| 'sn_last_error'
	| 'sn_alloc'
	| 'sn_free'
	| 'sn_task_status'
	| 'sn_task_status_address'
	| 'sn_task_error'
	| 'sn_task_result'
	| 'sn_task_output_count'
	| 'sn_task_output_data'
	| 'sn_task_output_size'
	| 'sn_task_release'
	| 'sn_debug_set_enabled'
	| 'sn_debug_enabled'
	| 'sn_debug_set_counter'
	| 'sn_debug_add_timing'
	| 'sn_debug_log'
	| 'sn_debug_snapshot'
	| 'sn_debug_reset_timings'
	| 'sn_debug_drain_log';

/** Runtime.h、Plugin.h、Physics.h、Texture.h、Navigation.h 里的函数。 */
export type RuntimeExport =
	| 'sn_nav_default_config'
	| 'sn_nav_build'
	| 'sn_nav_load'
	| 'sn_nav_serialize'
	| 'sn_nav_destroy'
	| 'sn_nav_find_path'
	| 'sn_crowd_create'
	| 'sn_crowd_destroy'
	| 'sn_crowd_add_agent'
	| 'sn_crowd_remove_agent'
	| 'sn_crowd_request_move'
	| 'sn_crowd_update'
	| 'sn_crowd_positions'
	| 'sn_physics_create'
	| 'sn_physics_destroy'
	| 'sn_physics_set_gravity'
	| 'sn_physics_create_mesh'
	| 'sn_physics_create_hull'
	| 'sn_physics_remove_bodies'
	| 'sn_physics_add_bodies'
	| 'sn_physics_step'
	| 'sn_physics_interpolate'
	| 'sn_physics_positions'
	| 'sn_physics_body_set_kinematic'
	| 'sn_physics_body_set_awake'
	| 'sn_physics_body_awake'
	| 'sn_physics_body_teleport'
	| 'sn_physics_body_set_rotation'
	| 'sn_physics_body_velocity'
	| 'sn_physics_body_set_velocity'
	| 'sn_physics_body_apply_impulse'
	| 'sn_physics_body_set_gravity'
	| 'sn_physics_body_set_never_sleep'
	| 'sn_physics_add_constraint'
	| 'sn_physics_remove_constraint'
	| 'sn_physics_body_set_contact_events'
	| 'sn_physics_body_set_sensor'
	| 'sn_physics_contact_events'
	| 'sn_physics_character_move'
	| 'sn_physics_character_grounded'
	| 'sn_physics_ray_cast'
	| 'sn_physics_ray_cast_all'
	| 'sn_physics_sphere_cast'
	| 'sn_physics_overlap'
	| 'sn_physics_static_triangles'
	| 'sn_plugin_abi_version'
	| 'sn_set_last_error'
	| 'sn_parallel_for'
	| 'sn_run_task'
	| 'sn_task_set_result'
	| 'sn_task_add_output'
	| 'sn_task_fail'
	| 'sn_on_shutdown'
	| 'sn_transforms_create'
	| 'sn_transforms_destroy'
	| 'sn_transforms_columns'
	| 'sn_transforms_alloc'
	| 'sn_transforms_free'
	| 'sn_transforms_set_parent'
	| 'sn_transforms_set_link'
	| 'sn_transforms_update'
	| 'sn_transforms_update_slot'
	| 'sn_transforms_slot_limit'
	| 'sn_transforms_cull'
	| 'sn_meshopt_decode'
	| 'sn_meshopt_decode_batch'
	| 'sn_anim_load_skeleton'
	| 'sn_anim_load_animation'
	| 'sn_anim_destroy_skeleton'
	| 'sn_anim_destroy_animation'
	| 'sn_anim_skeleton_joint_count'
	| 'sn_anim_animation_duration'
	| 'sn_anim_instance_create'
	| 'sn_anim_instance_destroy'
	| 'sn_anim_instance_model_matrices'
	| 'sn_anim_evaluate_batch'
	| 'sn_anim_skin_create'
	| 'sn_anim_skin_destroy'
	| 'sn_anim_skin_joint_count'
	| 'sn_anim_skin_batch'
	| 'sn_ktx2_get_info'
	| 'sn_ktx2_transcode';

/** Tools.h 加上 sn_nav_default_config 里的函数。 */
export type ToolsExport =
	| 'st_scene_load'
	| 'st_scene_destroy'
	| 'st_scene_mesh_info'
	| 'st_scene_material_name'
	| 'st_scene_mesh_copy'
	| 'st_scene_node_info'
	| 'st_scene_node_name'
	| 'st_scene_mesh_skin'
	| 'st_scene_mesh_morph_name'
	| 'st_scene_mesh_morph'
	| 'st_scene_animation_name'
	| 'st_scene_animation_bake'
	| 'st_generate_tangents'
	| 'st_meshopt_optimize'
	| 'st_meshopt_simplify'
	| 'st_meshopt_encode_vertex_bound'
	| 'st_meshopt_encode_index_bound'
	| 'st_meshopt_encode_vertices'
	| 'st_meshopt_encode_indices'
	| 'st_uv_unwrap'
	| 'st_convex_decompose'
	| 'st_image_decode'
	| 'st_image_resize'
	| 'st_cubemap_prefilter_ggx'
	| 'st_cubemap_sh_lambert'
	| 'st_basis_encode'
	| 'st_font_describe'
	| 'st_font_codepoints'
	| 'st_font_outlines'
	| 'st_font_msdf'
	| 'st_anim_build_skeleton'
	| 'st_anim_build_animation'
	| 'st_nav_bake'
	| 'sn_nav_default_config';
