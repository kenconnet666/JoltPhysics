// stride-native 运行时模块的 JS API（浏览器和 Node）。页面必须跨源隔离（COOP: same-origin，COEP: require-corp）。
import type { RuntimeExport } from './stride-native-core.mjs';
import {
	HANDLE, NAV_CONFIG_SIZE, NativeError, NativeModule, NativeObject, enumCode, instantiate, writeNavConfig,
	type Branded, type ModuleOptions, type NavConfig,
} from './native-module.ts';

export { NativeError, defaultWorkerThreads } from './native-module.ts';
export type { ModuleOptions, NativeDebugSnapshot, NativeErrorCode, NativeFeature, NativeLogLine, NavConfig } from './native-module.ts';

/** xyz；数组、元组和 Vector3 之类的 ArrayLike 都行。 */
export type Vec3 = ArrayLike<number>;
/** 写结果用的三个数（数组或 Vector3 之类）。 */
export type Vec3Out = { [index: number]: number };

/** 刚体在世界里的序号（按创建顺序），也是它在 positions() 里的下标。 */
export type BodyId = Branded<number, 'BodyId'>;
/** 代理在人群里的槽位，也是它在 Crowd.positions() 里的下标。 */
export type AgentId = Branded<number, 'AgentId'>;
/** 变换存储里的槽位，从 1 开始（0 是空槽位）；也是各列的下标：position[slot * 3]、world[slot * 16]。 */
export type TransformSlot = Branded<number, 'TransformSlot'>;

/** glTF EXT_meshopt_compression 的 mode。 */
export type MeshoptMode = 'attributes' | 'triangles' | 'indices';
/** glTF EXT_meshopt_compression 的 filter。 */
export type MeshoptFilter = 'none' | 'octahedral' | 'quaternion' | 'exponential' | 'color';
/** 转码目标，按上传时的 WebGPU 格式族命名。 */
export type TextureTarget = 'rgba8' | 'bc7' | 'astc4x4' | 'etc2rgba8';

const MESHOPT_MODES: Record<MeshoptMode, number> = { attributes: 0, triangles: 1, indices: 2 };
const MESHOPT_FILTERS: Record<MeshoptFilter, number> = { none: 0, octahedral: 1, quaternion: 2, exponential: 3, color: 4 };
const TEXTURE_TARGETS: Record<TextureTarget, number> = { rgba8: 0, bc7: 1, astc4x4: 2, etc2rgba8: 3 };

export interface MeshoptRequest {
	source: Uint8Array;
	count: number;
	stride: number;
	/** 默认 'attributes'。 */
	mode?: MeshoptMode;
	/** 默认 'none'。 */
	filter?: MeshoptFilter;
}

export interface Ktx2Info {
	width: number;
	height: number;
	levels: number;
	layers: number;
	faces: number;
	hasAlpha: boolean;
	isSrgb: boolean;
	isHdr: boolean;
}

export interface TranscodedTexture {
	width: number;
	height: number;
	/** 压缩格式是每个 4x4 块的字节数，rgba8 是每个像素的字节数。 */
	unitSize: number;
	/** 各级 mip，最大的在前。 */
	levels: Uint8Array[];
}

export interface AgentOptions {
	radius?: number;
	height?: number;
	maxSpeed?: number;
	maxAcceleration?: number;
}

/**
 * 加载运行时模块并启动任务系统。物理、贴图转码、导航是单独的侧模块，用 features 选用，比如
 * `createStrideNative({ features: [physics, navigation] })`；没选用的功能，调用它的方法会抛 NativeError。
 */
export async function createStrideNative({ debug = false, ...options }: ModuleOptions = {}): Promise<StrideNative> {
	// 动态导入：打包器把两个版本分开，页面只下载用到的那个。wasm 的地址也写成 new URL(..., import.meta.url)，
	// 打包器改写成和 Emscripten 胶水里同一个文件（见 instantiate）
	const core = debug
		? { factory: (await import('./stride-native-core-debug.mjs')).default, wasm: { file: 'stride-native-core-debug.wasm', url: new URL('./stride-native-core-debug.wasm', import.meta.url) } }
		: { factory: (await import('./stride-native-core.mjs')).default, wasm: { file: 'stride-native-core.wasm', url: new URL('./stride-native-core.wasm', import.meta.url) } };
	return new StrideNative(await instantiate(core.factory, { debug, ...options }, core.wasm));
}

export class StrideNative extends NativeModule<RuntimeExport> {
	/** 能放 capacity 个槽位的变换存储。各列一次分配好，视图一直有效。 */
	createTransforms(capacity: number): TransformStore {
		return new TransformStore(this, this.check(this.module._sn_transforms_create(capacity), 'sn_transforms_create'));
	}

	/** 需要 physics 功能。 */
	createPhysicsWorld(maxBodies = 1024, { transforms, ...settings }: PhysicsWorldOptions = {}): PhysicsWorld {
		this.requireFeature('physics', 'createPhysicsWorld');
		const handle = this.withScratch(SETTINGS_WORDS * 4, (table) => {
			writeSettings(this.f32(table, SETTINGS_WORDS), this.u32(table, SETTINGS_WORDS), settings);
			return this.module._sn_physics_create(maxBodies, transforms ? transforms[HANDLE] : 0, table);
		});
		return new PhysicsWorld(this, this.check(handle, 'sn_physics_create'), maxBodies);
	}

	/** 解码一个 glTF meshopt buffer view，返回 count * stride 个字节。 */
	decodeMeshopt(request: MeshoptRequest): Uint8Array {
		const { source, count, stride } = request;
		const mode = enumCode(MESHOPT_MODES, request.mode ?? 'attributes', 'MeshoptRequest.mode');
		const filter = enumCode(MESHOPT_FILTERS, request.filter ?? 'none', 'MeshoptRequest.filter');
		return this.withArrays([source], (src) => this.withScratch(count * stride, (dst) => {
			this.check(this.module._sn_meshopt_decode(dst, count, stride, src, source.byteLength, mode, filter), 'sn_meshopt_decode');
			return this.copyOut(Uint8Array, dst, count * stride);
		}));
	}

	/** 在任务系统上并行解码多个 buffer view。 */
	async decodeMeshoptBatch(requests: readonly MeshoptRequest[]): Promise<Uint8Array[]> {
		const REQUEST_SIZE = 32;
		const table = this.alloc(Math.max(1, requests.length) * REQUEST_SIZE);
		const buffers: number[] = [];
		try {
			requests.forEach((request, i) => {
				const mode = enumCode(MESHOPT_MODES, request.mode ?? 'attributes', `requests[${i}].mode`);
				const filter = enumCode(MESHOPT_FILTERS, request.filter ?? 'none', `requests[${i}].filter`);
				const src = this.copyIn(request.source);
				const dst = this.alloc(request.count * request.stride);
				buffers.push(src, dst);
				const view = this.view(table + i * REQUEST_SIZE, REQUEST_SIZE);
				view.setUint32(0, dst, true);
				view.setUint32(4, request.count, true);
				view.setUint32(8, request.stride, true);
				view.setUint32(12, src, true);
				view.setUint32(16, request.source.byteLength, true);
				view.setInt32(20, mode, true);
				view.setInt32(24, filter, true);
				view.setInt32(28, 1, true);
			});
			await this.awaitTask(this.module._sn_meshopt_decode_batch(table, requests.length), 'sn_meshopt_decode_batch');
			return requests.map((request, i) => this.copyOut(Uint8Array, buffers[i * 2 + 1]!, request.count * request.stride));
		} finally {
			for (const ptr of buffers) this.free(ptr);
			this.free(table);
		}
	}

	/** 需要 texture 功能。 */
	ktx2Info(bytes: Uint8Array): Ktx2Info {
		this.requireFeature('texture', 'ktx2Info');
		return this.withArrays([bytes], (data) => this.withScratch(32, (info) => {
			this.check(this.module._sn_ktx2_get_info(data, bytes.byteLength, info), 'sn_ktx2_get_info');
			const v = this.i32(info, 8);
			return { width: v[0]!, height: v[1]!, levels: v[2]!, layers: v[3]!, faces: v[4]!, hasAlpha: !!v[5], isSrgb: !!v[6], isHdr: !!v[7] };
		}));
	}

	/** 并行转码所有 mip 级。需要 texture 功能。 */
	async transcodeKtx2(bytes: Uint8Array, target: TextureTarget = 'rgba8'): Promise<TranscodedTexture> {
		this.requireFeature('texture', 'transcodeKtx2');
		const code = enumCode(TEXTURE_TARGETS, target, 'target');
		return this.withArraysAsync([bytes], (data) =>
			this.awaitTask(this.module._sn_ktx2_transcode(data, bytes.byteLength, code), 'sn_ktx2_transcode', (task) => ({
				width: task.result(0),
				height: task.result(1),
				unitSize: task.result(3),
				levels: Array.from({ length: task.result(2) }, (_, level) => task.output(Uint8Array, level)),
			})));
	}

	/** 从三角形汤构建分块导航网格，各 tile 并行。需要 navigation 功能。 */
	async buildNavMesh(vertices: Float32Array, triangles: Int32Array, config: NavConfig = {}): Promise<NavMesh> {
		this.requireFeature('navigation', 'buildNavMesh');
		return this.withArraysAsync([vertices, triangles], (v, t) => this.withScratch(NAV_CONFIG_SIZE, (cfg) => {
			writeNavConfig(this, cfg, config);
			return this.awaitTask(this.module._sn_nav_build(v, vertices.length / 3, t, triangles.length / 3, cfg), 'sn_nav_build',
				(task) => new NavMesh(this, task.result(0)));
		}));
	}

	/** 加载 NavMesh.serialize 或 stride-tools 烘焙出的导航网格。需要 navigation 功能。 */
	loadNavMesh(bytes: Uint8Array): NavMesh {
		this.requireFeature('navigation', 'loadNavMesh');
		return this.withArrays([bytes], (data) => new NavMesh(this, this.check(this.module._sn_nav_load(data, bytes.byteLength), 'sn_nav_load')));
	}

	loadSkeleton(ozzArchive: Uint8Array): Skeleton {
		return this.withArrays([ozzArchive], (data) =>
			new Skeleton(this, this.check(this.module._sn_anim_load_skeleton(data, ozzArchive.byteLength), 'sn_anim_load_skeleton')));
	}

	loadAnimation(ozzArchive: Uint8Array): Animation {
		return this.withArrays([ozzArchive], (data) =>
			new Animation(this, this.check(this.module._sn_anim_load_animation(data, ozzArchive.byteLength), 'sn_anim_load_animation')));
	}

	/** 蒙皮：joints 是骨架的关节下标，inverseBinds 每个关节一个列主序 4x4（网格到绑定时的关节空间）。 */
	createSkin(joints: Uint32Array, inverseBinds: Float32Array): Skin {
		if (inverseBinds.length !== joints.length * 16) throw new NativeError(`createSkin: ${joints.length} joints need ${joints.length * 16} inverse bind floats, got ${inverseBinds.length}`, -1);
		return this.withArrays([joints, inverseBinds], (j, b) => new Skin(this, this.check(this.module._sn_anim_skin_create(j, b, joints.length), 'sn_anim_skin_create'), joints.length));
	}
}

// sn_shape：type、position[3]、rotation[4]、size[3]、mesh、mass，13 个字
const SHAPE_WORDS = 13;
// sn_body：motion、flags、transform、position[3]、rotation[4]、first_shape、shape_count、layer、friction、character，15 个字
const BODY_WORDS = 15;
// sn_character：6 个 float
const CHARACTER_WORDS = 6;
// sn_hit：distance、point[3]、normal[3]、body
const HIT_WORDS = 8;
// sn_physics_settings：gravity[3]、linear_damping、angular_damping、collision_masks[32]
const SETTINGS_WORDS = 37;

const SHAPE_TYPES = { box: 0, sphere: 1, capsule: 2, cylinder: 3, mesh: 4, convexHull: 5 } as const;
const MOTIONS = { static: 0, dynamic: 1, kinematic: 2, character: 3 } as const;
const BODY_INTERPOLATED = 1;
const BODY_NO_GRAVITY = 2;
const BODY_NEVER_SLEEP = 4;
const ALL_LAYERS = 0xffffffff;

const ORIGIN: Vec3 = [0, 0, 0];
const IDENTITY: ArrayLike<number> = [0, 0, 0, 1];

/** 用 PhysicsWorld.createMesh 建的三角网格形状，同一个世界里的物体共用。 */
export type PhysicsMesh = Branded<number, 'PhysicsMesh'>;
/** 用 PhysicsWorld.createHull 建的凸包形状，同一个世界里的物体共用。 */
export type PhysicsHull = Branded<number, 'PhysicsHull'>;

/** 复合碰撞体里的一个形状（Stride 的 BoxCollider、SphereCollider……加 CompoundCollider 的本地位姿）。 */
export type ShapeDesc = (
	| { type: 'box'; /** 边长（Stride 的 BoxCollider.Size）。 */ size: Vec3 }
	| { type: 'sphere'; radius: number }
	/** 沿本地 Y；length 是两端球心的距离（Stride 的 CapsuleCollider.Length）。 */
	| { type: 'capsule'; radius: number; length: number }
	| { type: 'cylinder'; radius: number; length: number }
	/** 按实体的世界缩放缩放（Stride 的 MeshCollider）；只能用在静态和运动学物体上。 */
	| { type: 'mesh'; mesh: PhysicsMesh }
	/** 按实体的世界缩放缩放（Stride 的 ConvexHullCollider）。 */
	| { type: 'convexHull'; hull: PhysicsHull }
) & {
	/** 本地位置，默认原点。 */
	position?: Vec3;
	/** 本地旋转（xyzw），默认单位。 */
	rotation?: ArrayLike<number>;
	/** 动态物体和角色的质量是各形状的和，默认 1。 */
	mass?: number;
};

/** 角色参数（Stride 的 CharacterComponent），默认值同 Stride。 */
export interface CharacterSettings {
	maximumHorizontalForce?: number;
	maximumVerticalForce?: number;
	/** Stride 的 JumpForce：起跳时相对支撑物的竖直速度。 */
	jumpVelocity?: number;
	/** 能站立的最大坡度（度）。 */
	slopeAngle?: number;
	airControlForceScale?: number;
	airControlScale?: number;
}

export interface BodyDesc {
	/** 默认 'static'；'character' 是 Stride 的 CharacterComponent：不转动，用 moveCharacter 控制。 */
	motion?: keyof typeof MOTIONS;
	shapes: readonly ShapeDesc[];
	/**
	 * 绑定的变换槽位（世界要用 createPhysicsWorld(n, { transforms }) 创建）。初始位姿取槽位当前的世界位姿（缩放只作用到网格形状），
	 * 动态物体和角色每步结束把位姿写回槽位的本地位置和旋转，有父节点时换算到父节点空间。物体存在期间槽位不能释放。
	 */
	transform?: TransformSlot;
	/** 没有绑定变换时的位姿。 */
	position?: Vec3;
	rotation?: ArrayLike<number>;
	/** Stride 的 CollisionLayer，0..31，默认 0。 */
	layer?: number;
	/** Stride 的 FrictionCoefficient，默认 1。 */
	friction?: number;
	/** InterpolationMode.Interpolated：interpolate(alpha) 写回两步之间的位姿。 */
	interpolated?: boolean;
	/** 默认 true。 */
	gravity?: boolean;
	/** 一直醒着（Stride 的 SleepThreshold 为负）。 */
	neverSleep?: boolean;
	/** motion 为 'character' 时用。 */
	character?: CharacterSettings;
}

export interface BoxDesc {
	/** 初始位置，默认原点；绑定了 transform 时不用它。 */
	position?: Vec3;
	halfExtent: Vec3;
	/** 默认 'dynamic'。 */
	motion?: 'dynamic' | 'static';
	transform?: TransformSlot;
}

/** 仿真设置（Stride 的 BepuSimulation），默认值同 Stride。 */
export interface PhysicsSettings {
	/** 默认 (0, -9.8, 0)；阻尼默认各 0.05（Stride 的 StridePoseIntegratorCallbacks）。 */
	gravity?: Vec3;
	linearDamping?: number;
	angularDamping?: number;
	/** 碰撞矩阵：collisionMasks[a] 的第 b 位为 1 表示层 a 和层 b 相互碰撞，要对称；默认全部碰撞。 */
	collisionMasks?: ArrayLike<number>;
}

export interface PhysicsWorldOptions extends PhysicsSettings {
	/** 物体可以绑定的变换存储；它要比世界活得久。 */
	transforms?: TransformStore;
}

export interface QueryOptions {
	/** 第 n 位为 1 表示层 n 的物体参与（Stride 的 CollisionMask），默认全部。 */
	mask?: number;
	/** 结果写进这个对象（就地改写，不分配内存）。 */
	hit?: PhysicsHit;
}

export interface SweepOptions extends QueryOptions {
	/**
	 * 忽略起点就和形状重叠的物体（默认它们算距离 0 的命中，同 Bepu）。第三人称相机用：贴着旁边的箱子时不该把相机拉到
	 * 最近；要防止相机钻进正后方的墙，再配一条射线。
	 */
	ignoreInitialOverlap?: boolean;
}

const CAST_IGNORE_INITIAL_OVERLAP = 1;

/** 射线和扫掠的结果；查询传入一个对象时就地改写它，不分配内存。 */
export class PhysicsHit {
	/** 沿方向的距离；球体扫掠从一开始就重叠时是 0。 */
	distance = 0;
	readonly point: [number, number, number] = [0, 0, 0];
	readonly normal: [number, number, number] = [0, 0, 0];
	body = 0 as BodyId;
}

/** 用 StrideNative.createPhysicsWorld 创建。 */
export class PhysicsWorld extends NativeObject<StrideNative> {
	readonly maxBodies: number;
	#bodyCount = 0;
	readonly #positionsAddress: number;
	#positions: Float32Array;
	// 查询复用的 origin、direction 和 sn_hit，查询不分配内存
	readonly #query: number;
	readonly #queryFloats: Float32Array;

	/** @internal */
	constructor(native: StrideNative, handle: number, maxBodies: number) {
		super(native, handle);
		this.maxBodies = maxBodies;
		this.#positionsAddress = native.module._sn_physics_positions(handle);
		this.#positions = native.f32(this.#positionsAddress, 0);
		this.#query = native.alloc((6 + HIT_WORDS) * 4);
		this.#queryFloats = native.f32(this.#query, 6 + HIT_WORDS);
	}

	get bodyCount(): number {
		return this.#bodyCount;
	}

	/** 三角网格形状：positions 是 xyz，indices 每三个一个三角形（逆时针为正面）。 */
	createMesh(positions: Float32Array, indices: Uint32Array): PhysicsMesh {
		const n = this.native;
		return n.withArrays([positions, indices], (p, i) =>
			n.check(n.module._sn_physics_create_mesh(this[HANDLE], p, positions.length / 3, i, indices.length), 'sn_physics_create_mesh') as PhysicsMesh);
	}

	/** 凸包形状：Jolt 由这些点（xyz）算出凸包，至少 4 个点。 */
	createHull(points: Float32Array): PhysicsHull {
		const n = this.native;
		return n.withArrays([points], (p) => n.check(n.module._sn_physics_create_hull(this[HANDLE], p, points.length / 3), 'sn_physics_create_hull') as PhysicsHull);
	}

	addBody(body: BodyDesc): BodyId {
		return this.addBodies([body])[0]!;
	}

	/** 一次原生调用加入所有物体，Jolt 成批插进宽相。返回值和 bodies 一一对应。 */
	addBodies(bodies: readonly BodyDesc[]): BodyId[] {
		if (bodies.length === 0) return [];
		const n = this.native;
		const shapeCount = bodies.reduce((sum, body) => sum + body.shapes.length, 0);
		const characters = bodies.filter((body) => body.motion === 'character');
		const bytes = (bodies.length * BODY_WORDS + shapeCount * SHAPE_WORDS + characters.length * CHARACTER_WORDS) * 4;
		return n.withScratch(bytes, (table) => {
			const floats = n.f32(table, bytes / 4);
			const words = n.u32(table, bytes / 4);
			const shapesWord = bodies.length * BODY_WORDS;
			const charactersWord = shapesWord + shapeCount * SHAPE_WORDS;
			let shape = 0, character = 0;
			bodies.forEach((body, i) => {
				body.shapes.forEach((desc, s) => writeShape(floats, words, shapesWord + (shape + s) * SHAPE_WORDS, desc, i, s));
				if (body.motion === 'character') writeCharacter(floats, charactersWord + character * CHARACTER_WORDS, body.character ?? {});
				writeBody(floats, words, i * BODY_WORDS, body, i, shape, body.motion === 'character' ? character++ : 0);
				shape += body.shapes.length;
			});
			const first = n.check(n.module._sn_physics_add_bodies(this[HANDLE], table, bodies.length, table + shapesWord * 4, shapeCount,
				table + charactersWord * 4, characters.length), 'sn_physics_add_bodies');
			this.#bodyCount = first + bodies.length;
			this.#positions = n.f32(this.#positionsAddress, this.#bodyCount * 3);
			return Array.from({ length: bodies.length }, (_, i) => (first + i) as BodyId);
		});
	}

	/** 一个盒子一个物体的简写（默认动态）。 */
	addBox(box: BoxDesc): BodyId {
		return this.addBoxes([box])[0]!;
	}

	addBoxes(boxes: readonly BoxDesc[]): BodyId[] {
		return this.addBodies(boxes.map(({ position, halfExtent, motion = 'dynamic', transform }, i) => ({
			motion: boxMotion(motion, i),
			shapes: [{ type: 'box', size: [halfExtent[0]! * 2, halfExtent[1]! * 2, halfExtent[2]! * 2] }],
			...(position ? { position } : {}),
			...(transform ? { transform } : {}),
		})));
	}

	/** 步进一个固定步长（在共享任务系统上，当前线程等待时也跑任务）：角色逻辑、Jolt 步进、写回位姿，然后刷新 positions()。 */
	step(deltaTime: number, collisionSteps = 1): void {
		this.native.check(this.native.module._sn_physics_step(this[HANDLE], deltaTime, collisionSteps), 'sn_physics_step');
	}

	/** 插值的物体写回 alpha（0 是上一步，1 是这一步）处的位姿。 */
	interpolate(alpha: number): void {
		this.native.check(this.native.module._sn_physics_interpolate(this[HANDLE], alpha), 'sn_physics_interpolate');
	}

	/**
	 * 每个物体的位置 xyz，按 BodyId 排：原生内存上的视图，step() 原地改写，所以不拷贝，加物体之前一直是同一个对象。
	 * 世界销毁后失效。
	 */
	positions(): Float32Array {
		return this.#positions;
	}

	/** 在动态和运动学之间切换（Stride 的 BodyComponent.Kinematic）；只对按 'dynamic' 或 'kinematic' 加入的物体有效。 */
	setKinematic(body: BodyId, kinematic: boolean): void {
		this.native.check(this.native.module._sn_physics_body_set_kinematic(this[HANDLE], body, kinematic ? 1 : 0), 'sn_physics_body_set_kinematic');
	}

	/** 唤醒或让它睡下（Stride 的 Awake）。 */
	setAwake(body: BodyId, awake: boolean): void {
		this.native.check(this.native.module._sn_physics_body_set_awake(this[HANDLE], body, awake ? 1 : 0), 'sn_physics_body_set_awake');
	}

	isAwake(body: BodyId): boolean {
		return this.native.check(this.native.module._sn_physics_body_awake(this[HANDLE], body), 'sn_physics_body_awake') === 1;
	}

	/** 瞬移到世界位姿并唤醒（Stride 的 Teleport）；绑定了变换的立即写回。 */
	teleport(body: BodyId, position: Vec3, rotation: ArrayLike<number>): void {
		const f = this.#queryFloats;
		f.set([position[0]!, position[1]!, position[2]!, rotation[0]!, rotation[1]!, rotation[2]!, rotation[3]!]);
		const q = this.#query;
		this.native.check(this.native.module._sn_physics_body_teleport(this[HANDLE], body, q, q + 12), 'sn_physics_body_teleport');
	}

	/** 线速度和角速度（世界空间），写进 linear、angular（各 3 个数）。 */
	velocity(body: BodyId, linear: Vec3Out, angular: Vec3Out): void {
		const q = this.#query;
		this.native.check(this.native.module._sn_physics_body_velocity(this[HANDLE], body, q, q + 12), 'sn_physics_body_velocity');
		const f = this.#queryFloats;
		linear[0] = f[0]!; linear[1] = f[1]!; linear[2] = f[2]!;
		angular[0] = f[3]!; angular[1] = f[4]!; angular[2] = f[5]!;
	}

	/** 设线速度、角速度；不给的不变。 */
	setVelocity(body: BodyId, linear?: Vec3, angular?: Vec3): void {
		this.#pair(linear, angular, (l, a) => this.native.module._sn_physics_body_set_velocity(this[HANDLE], body, l, a), 'sn_physics_body_set_velocity');
	}

	/** 在质心加冲量（Stride 的 ApplyLinearImpulse、ApplyAngularImpulse）；运动学物体忽略。 */
	applyImpulse(body: BodyId, linear?: Vec3, angular?: Vec3): void {
		this.#pair(linear, angular, (l, a) => this.native.module._sn_physics_body_apply_impulse(this[HANDLE], body, l, a), 'sn_physics_body_apply_impulse');
	}

	/** 受不受重力（Stride 的 Gravity）。 */
	setGravity(body: BodyId, enabled: boolean): void {
		this.native.check(this.native.module._sn_physics_body_set_gravity(this[HANDLE], body, enabled ? 1 : 0), 'sn_physics_body_set_gravity');
	}

	/** 两个可选的向量放进查询缓冲（没给的传空指针）调用 call。 */
	#pair(first: Vec3 | undefined, second: Vec3 | undefined, call: (first: number, second: number) => number, what: string): void {
		const f = this.#queryFloats, q = this.#query;
		if (first) f.set([first[0]!, first[1]!, first[2]!], 0);
		if (second) f.set([second[0]!, second[1]!, second[2]!], 3);
		this.native.check(call(first ? q : 0, second ? q + 12 : 0), what);
	}

	/**
	 * 设角色下一步的移动（Stride 的 MoveVector · Speed 和 TryJump）：target 是世界 X、Z 方向的目标速度；
	 * jump 为真时试着起跳，只在站稳时生效，下一步用掉。
	 */
	moveCharacter(body: BodyId, targetX: number, targetZ: number, jump = false): void {
		this.native.check(this.native.module._sn_physics_character_move(this[HANDLE], body, targetX, targetZ, jump ? 1 : 0), 'sn_physics_character_move');
	}

	/** 角色是否站在能站立的地面上（Stride 的 IsGrounded），按上一步结束时的接触算。 */
	isGrounded(body: BodyId): boolean {
		return this.native.check(this.native.module._sn_physics_character_grounded(this[HANDLE], body), 'sn_physics_character_grounded') === 1;
	}

	/** 射线（Stride 的 RayCast）。没命中时返回 undefined。 */
	rayCast(origin: Vec3, direction: Vec3, maxDistance: number, { mask = ALL_LAYERS, hit = new PhysicsHit() }: QueryOptions = {}): PhysicsHit | undefined {
		this.#setQuery(origin, direction);
		const q = this.#query;
		const result = this.native.module._sn_physics_ray_cast(this[HANDLE], q, q + 12, maxDistance, mask >>> 0, q + 24);
		return this.#hit(this.native.check(result, 'sn_physics_ray_cast'), hit);
	}

	/** 球体扫掠（Stride 的 SweepCast 加 Sphere），同 rayCast；默认从一开始就重叠的物体算距离 0 的命中（同 Bepu）。 */
	sphereCast(radius: number, origin: Vec3, direction: Vec3, maxDistance: number, { mask = ALL_LAYERS, hit = new PhysicsHit(), ignoreInitialOverlap = false }: SweepOptions = {}): PhysicsHit | undefined {
		this.#setQuery(origin, direction);
		const q = this.#query;
		const flags = ignoreInitialOverlap ? CAST_IGNORE_INITIAL_OVERLAP : 0;
		const result = this.native.module._sn_physics_sphere_cast(this[HANDLE], radius, q, q + 12, maxDistance, mask >>> 0, flags, q + 24);
		return this.#hit(this.native.check(result, 'sn_physics_sphere_cast'), hit);
	}

	#setQuery(origin: Vec3, direction: Vec3): void {
		const f = this.#queryFloats;
		f[0] = origin[0]!; f[1] = origin[1]!; f[2] = origin[2]!;
		f[3] = direction[0]!; f[4] = direction[1]!; f[5] = direction[2]!;
	}

	#hit(found: number, hit: PhysicsHit): PhysicsHit | undefined {
		if (found === 0) return undefined;
		const f = this.#queryFloats;
		hit.distance = f[6]!;
		hit.point[0] = f[7]!; hit.point[1] = f[8]!; hit.point[2] = f[9]!;
		hit.normal[0] = f[10]!; hit.normal[1] = f[11]!; hit.normal[2] = f[12]!;
		hit.body = this.native.i32(this.#query + 13 * 4, 1)[0]! as BodyId;
		return hit;
	}

	protected override release(): void {
		this.native.module._sn_physics_destroy(this[HANDLE]);
		this.native.free(this.#query);
	}
}

function boxMotion(motion: string, index: number): 'dynamic' | 'static' {
	if (motion !== 'dynamic' && motion !== 'static') throw new TypeError(`box ${index}: motion must be 'dynamic' or 'static', not '${motion}'`);
	return motion;
}

/** createPhysicsWorld 的设置写成 sn_physics_settings。 */
function writeSettings(floats: Float32Array, words: Uint32Array, { gravity = [0, -9.8, 0], linearDamping = 0.05, angularDamping = 0.05, collisionMasks }: PhysicsSettings): void {
	floats.set([gravity[0]!, gravity[1]!, gravity[2]!, linearDamping, angularDamping]);
	if (collisionMasks && collisionMasks.length !== 32) throw new RangeError(`collisionMasks needs 32 entries, got ${collisionMasks.length}`);
	for (let layer = 0; layer < 32; layer++) words[5 + layer] = collisionMasks ? collisionMasks[layer]! >>> 0 : ALL_LAYERS;
}

function writeShape(floats: Float32Array, words: Uint32Array, o: number, shape: ShapeDesc, body: number, index: number): void {
	const type = SHAPE_TYPES[shape.type];
	if (type === undefined) throw new TypeError(`body ${body} shape ${index}: unknown type '${String(shape.type)}'`);
	words[o] = type;
	const position = shape.position ?? ORIGIN, rotation = shape.rotation ?? IDENTITY;
	floats.set([position[0]!, position[1]!, position[2]!, rotation[0]!, rotation[1]!, rotation[2]!, rotation[3]!], o + 1);
	const size = shape.type === 'box' ? [shape.size[0]!, shape.size[1]!, shape.size[2]!]
		: shape.type === 'sphere' ? [shape.radius, 0, 0]
			: shape.type === 'mesh' || shape.type === 'convexHull' ? [0, 0, 0] : [shape.radius, shape.length, 0];
	floats.set(size, o + 8);
	words[o + 11] = shape.type === 'mesh' ? shape.mesh : shape.type === 'convexHull' ? shape.hull : 0;
	floats[o + 12] = shape.mass ?? 1;
}

function writeCharacter(floats: Float32Array, o: number, settings: CharacterSettings): void {
	floats.set([
		settings.maximumHorizontalForce ?? 200, settings.maximumVerticalForce ?? 1000, settings.jumpVelocity ?? 10, settings.slopeAngle ?? 50,
		settings.airControlForceScale ?? 0.2, settings.airControlScale ?? 0.2,
	], o);
}

function writeBody(floats: Float32Array, words: Uint32Array, o: number, body: BodyDesc, index: number, firstShape: number, character: number): void {
	const motion = MOTIONS[body.motion ?? 'static'];
	if (motion === undefined) throw new TypeError(`body ${index}: motion must be 'static', 'dynamic', 'kinematic' or 'character', not '${String(body.motion)}'`);
	if (body.shapes.length === 0) throw new RangeError(`body ${index} needs at least one shape`);
	const layer = body.layer ?? 0;
	if (!Number.isInteger(layer) || layer < 0 || layer > 31) throw new RangeError(`body ${index}: layer must be an integer 0..31, got ${layer}`);
	words[o] = motion;
	words[o + 1] = (body.interpolated ? BODY_INTERPOLATED : 0) | (body.gravity === false ? BODY_NO_GRAVITY : 0) | (body.neverSleep ? BODY_NEVER_SLEEP : 0);
	words[o + 2] = body.transform ?? 0;
	const position = body.position ?? ORIGIN, rotation = body.rotation ?? IDENTITY;
	floats.set([position[0]!, position[1]!, position[2]!, rotation[0]!, rotation[1]!, rotation[2]!, rotation[3]!], o + 3);
	words[o + 10] = firstShape;
	words[o + 11] = body.shapes.length;
	words[o + 12] = layer;
	floats[o + 13] = body.friction ?? 1;
	words[o + 14] = character;
}


/** sn_transform_columns：5 个指针加 capacity。 */
const COLUMNS_SIZE = 24;

/**
 * 实体的本地位姿和世界矩阵，按结构数组放在模块内存里，用 StrideNative.createTransforms 创建。
 * 列是原生内存上的常驻视图：直接读写本地位置、旋转、缩放，update() 之后读世界矩阵，世界矩阵也能整段交给
 * GPUQueue.writeBuffer。物理世界绑定了槽位时，步进会把位姿写回这里。
 */
export class TransformStore extends NativeObject<StrideNative> {
	/** 最大槽位号；槽位是 1..capacity。 */
	readonly capacity: number;
	/** 本地位置，position[slot * 3 + 0..2]。 */
	readonly position: Float32Array;
	/** 本地旋转，单位四元数 xyzw，rotation[slot * 4 + 0..3]。 */
	readonly rotation: Float32Array;
	/** 本地缩放，scale[slot * 3 + 0..2]。 */
	readonly scale: Float32Array;
	/** 父槽位，0 表示根。只能读：要改用 setParent，原生那边要维护子节点数和计算顺序。 */
	readonly parent: Int32Array;
	/** 世界矩阵，列主序 4x4，world[slot * 16 + 0..15]，平移在 12..14；update() 写入。 */
	readonly world: Float32Array;
	readonly #scratch: number;

	/** @internal */
	constructor(native: StrideNative, handle: number) {
		super(native, handle);
		const [position, rotation, scale, parent, world, capacity] = native.withScratch(COLUMNS_SIZE, (out) => {
			native.check(native.module._sn_transforms_columns(handle, out), 'sn_transforms_columns');
			return Array.from(native.u32(out, 6));
		}) as [number, number, number, number, number, number];
		const slots = capacity + 1;
		this.capacity = capacity;
		this.position = native.f32(position, slots * 3);
		this.rotation = native.f32(rotation, slots * 4);
		this.scale = native.f32(scale, slots * 3);
		this.parent = native.i32(parent, slots);
		this.world = native.f32(world, slots * 16);
		this.#scratch = native.alloc(4);
	}

	/** 分配一个槽位：位置 0、旋转单位、缩放 1、没有父节点。容量不够时抛 NativeError。 */
	allocate(): TransformSlot;
	/** 一次分配 count 个槽位；容量不够时一个都不分配。 */
	allocate(count: number): TransformSlot[];
	allocate(count?: number): TransformSlot | TransformSlot[] {
		const n = this.native;
		if (count === undefined) {
			n.check(n.module._sn_transforms_alloc(this[HANDLE], this.#scratch, 1), 'sn_transforms_alloc');
			return n.i32(this.#scratch, 1)[0] as TransformSlot;
		}
		if (count === 0) return [];
		return n.withScratch(count * 4, (out) => {
			n.check(n.module._sn_transforms_alloc(this[HANDLE], out, count), 'sn_transforms_alloc');
			return Array.from(n.i32(out, count)) as TransformSlot[];
		});
	}

	/** 释放槽位。还有子节点留着的槽位不能释放（一起释放的子节点不算），这时一个都不释放。 */
	free(slots: TransformSlot | readonly TransformSlot[]): void {
		const n = this.native;
		if (typeof slots === 'number') {
			n.i32(this.#scratch, 1)[0] = slots;
			n.check(n.module._sn_transforms_free(this[HANDLE], this.#scratch, 1), 'sn_transforms_free');
			return;
		}
		if (slots.length === 0) return;
		n.withArrays([Int32Array.from(slots)], (pointer) =>
			n.check(n.module._sn_transforms_free(this[HANDLE], pointer, slots.length), 'sn_transforms_free'));
	}

	/** 把 slot 挂到 parent 下；null 表示变成根。不能形成环。 */
	setParent(slot: TransformSlot, parent: TransformSlot | null): void {
		this.native.check(this.native.module._sn_transforms_set_parent(this[HANDLE], slot, parent ?? 0), 'sn_transforms_set_parent');
	}

	/** 发出去过的最大槽位号（释放了也不降）：用到的槽位都在 1..slotLimit 之内，上传世界矩阵时只传 world 的前 (slotLimit + 1) * 16 个 float。 */
	get slotLimit(): number {
		return this.native.check(this.native.module._sn_transforms_slot_limit(this[HANDLE]), 'sn_transforms_slot_limit');
	}

	/** 按父节点在前的顺序算所有世界矩阵。层级变了才重排顺序，平时是一遍顺序扫描。 */
	update(): void {
		this.native.check(this.native.module._sn_transforms_update(this[HANDLE]), 'sn_transforms_update');
	}

	/** 这个存储的视锥剔除工作区；渲染器每帧复用一个。 */
	createCuller(): FrustumCuller {
		return new FrustumCuller(this.native, this);
	}

	protected override release(): void {
		this.native.module._sn_transforms_destroy(this[HANDLE]);
		this.native.free(this.#scratch);
	}
}

/** sn_cull_view：6 个平面（24 个 float）、plane_count、measure_distance、distance_plane（4 个 float）、min、max。 */
const CULL_VIEW_WORDS = 32;
const MAX_CULL_VIEWS = 8;

/**
 * 视锥剔除（sn_transforms_cull）的工作区：输入和输出都在原生内存里，属性是常驻的视图，每帧直接写、直接读，不分配。
 * 用法：reserve 容量 → 往 slots 按批次连续写槽位、batchEnds 写每批的结束下标、bounds 写每批的本地包围盒（中心、半长）
 * → setView 写各视图的平面 → cull → 读 counts、visibleSlots 和视图的距离范围。
 */
export class FrustumCuller implements Disposable {
	readonly #native: StrideNative;
	readonly #transforms: TransformStore;
	#slotCapacity = 0;
	#batchCapacity = 0;
	// 一次分配：slots、batchEnds、bounds、views、visibleSlots、counts
	#block = 0;
	#slots: Int32Array = new Int32Array(0);
	#batchEnds: Uint32Array = new Uint32Array(0);
	#bounds: Float32Array = new Float32Array(0);
	#views: Float32Array = new Float32Array(0);
	#viewWords: Int32Array = new Int32Array(0);
	#visibleSlots: Int32Array = new Int32Array(0);
	#counts: Uint32Array = new Uint32Array(0);
	#viewsAddress = 0;
	#destroyed = false;

	/** @internal 用 TransformStore.createCuller 创建。 */
	constructor(native: StrideNative, transforms: TransformStore) {
		this.#native = native;
		this.#transforms = transforms;
	}

	/** 实例的槽位，按批次连续排。 */
	get slots(): Int32Array { return this.#slots; }
	/** 每批的结束下标（不含）。 */
	get batchEnds(): Uint32Array { return this.#batchEnds; }
	/** 每批 6 个 float：本地包围盒的中心 xyz、半长 xyz。半长为 0 的总是可见。 */
	get bounds(): Float32Array { return this.#bounds; }
	/** 上一次 cull 的结果：第 v 个视图的可见槽位在 v * slotCount 起的一段里，每批放在这批原来的起点。 */
	get visibleSlots(): Int32Array { return this.#visibleSlots; }
	/** 上一次 cull 的结果：第 v 个视图第 b 批的可见个数在 v * batchCount + b。 */
	get counts(): Uint32Array { return this.#counts; }
	get slotCapacity(): number { return this.#slotCapacity; }
	get batchCapacity(): number { return this.#batchCapacity; }

	/** 保证能放这么多实例和批次；容量变了就重新分配，之前取到的属性数组不能再用（内容不保留）。 */
	reserve(slotCount: number, batchCount: number): void {
		if (this.#destroyed) throw new NativeError('FrustumCuller was destroyed', -2);
		if (slotCount <= this.#slotCapacity && batchCount <= this.#batchCapacity) return;
		const slots = Math.max(slotCount, this.#slotCapacity * 2, 64);
		const batches = Math.max(batchCount, this.#batchCapacity * 2, 16);
		const n = this.#native;
		if (this.#block) n.free(this.#block);
		// 各段按 4 字节对齐，sn_cull_view 只有 4 字节的成员
		const words = slots + batches + batches * 6 + MAX_CULL_VIEWS * CULL_VIEW_WORDS + MAX_CULL_VIEWS * slots + MAX_CULL_VIEWS * batches;
		const block = (this.#block = n.alloc(words * 4));
		let offset = block;
		const take = <A extends Int32Array | Uint32Array | Float32Array>(type: { new (buffer: ArrayBufferLike, byteOffset: number, length: number): A }, length: number): A => {
			const view = new type(n.buffer, offset, length);
			offset += length * 4;
			return view;
		};
		this.#slots = take(Int32Array, slots);
		this.#batchEnds = take(Uint32Array, batches);
		this.#bounds = take(Float32Array, batches * 6);
		this.#viewsAddress = offset;
		this.#views = take(Float32Array, MAX_CULL_VIEWS * CULL_VIEW_WORDS);
		this.#viewWords = new Int32Array(n.buffer, this.#viewsAddress, MAX_CULL_VIEWS * CULL_VIEW_WORDS);
		this.#visibleSlots = take(Int32Array, MAX_CULL_VIEWS * slots);
		this.#counts = take(Uint32Array, MAX_CULL_VIEWS * batches);
		this.#slotCapacity = slots;
		this.#batchCapacity = batches;
	}

	/**
	 * 写第 index 个视图（最多 8 个）：planes 是 6 个朝内的平面（xyzw，dot(n, p) + w > 0 在内侧，近、远平面在最后），
	 * planeCount 是 6 或 4（4 忽略近、远平面）；给了 distancePlane（xyz 法线、w，距离 = dot(n, p) - w）就量出可见物体的距离范围。
	 */
	setView(index: number, planes: ArrayLike<number>, planeCount: 4 | 6, distancePlane?: ArrayLike<number>): void {
		if (!Number.isInteger(index) || index < 0 || index >= MAX_CULL_VIEWS) throw new RangeError(`view index must be 0..${MAX_CULL_VIEWS - 1}, got ${index}`);
		if (planes.length < planeCount * 4) throw new RangeError(`planes must hold ${planeCount * 4} numbers, got ${planes.length}`);
		const o = index * CULL_VIEW_WORDS;
		for (let i = 0; i < 24; i++) this.#views[o + i] = i < planeCount * 4 ? planes[i]! : 0;
		this.#viewWords[o + 24] = planeCount;
		this.#viewWords[o + 25] = distancePlane ? 1 : 0;
		for (let i = 0; i < 4; i++) this.#views[o + 26 + i] = distancePlane?.[i] ?? 0;
	}

	/** 剔除前 viewCount 个视图。batchEnds 的最后一个必须等于 slotCount。 */
	cull(slotCount: number, batchCount: number, viewCount: number): void {
		const n = this.#native;
		if (slotCount > this.#slotCapacity || batchCount > this.#batchCapacity) throw new RangeError('FrustumCuller: reserve before writing more slots or batches');
		if (viewCount > MAX_CULL_VIEWS) throw new RangeError(`FrustumCuller: at most ${MAX_CULL_VIEWS} views, got ${viewCount}`);
		n.check(n.module._sn_transforms_cull(n.handleOf(this.#transforms), this.#slots.byteOffset, slotCount, this.#batchEnds.byteOffset, this.#bounds.byteOffset, batchCount,
			this.#viewsAddress, viewCount, this.#visibleSlots.byteOffset, this.#counts.byteOffset), 'sn_transforms_cull');
	}

	/** 第 index 个视图里可见物体到距离平面的最近距离；没有可见物体时是 +Infinity。 */
	minDistance(index: number): number { return this.#views[index * CULL_VIEW_WORDS + 30]!; }
	maxDistance(index: number): number { return this.#views[index * CULL_VIEW_WORDS + 31]!; }

	destroy(): void {
		if (this.#destroyed) return;
		this.#destroyed = true;
		if (this.#block) this.#native.free(this.#block);
		this.#block = 0;
	}

	[Symbol.dispose](): void {
		this.destroy();
	}
}

export class NavMesh extends NativeObject<StrideNative> {
	/** 从 start 到 end 的拐点（xyz）；没有路径时抛 NativeError。 */
	findPath(start: Vec3, end: Vec3, searchExtents: Vec3 = [2, 4, 2], maxPoints = 256): Float32Array {
		const n = this.native;
		return n.withArrays([new Float32Array(start), new Float32Array(end), new Float32Array(searchExtents)], (s, e, x) =>
			n.withScratch(maxPoints * 12, (out) => {
				const count = n.check(n.module._sn_nav_find_path(this[HANDLE], s, e, x, out, maxPoints), 'sn_nav_find_path');
				return n.copyOut(Float32Array, out, count * 3);
			}));
	}

	serialize(): Uint8Array {
		const n = this.native;
		return n.readOwnedBuffer((data, size) => n.module._sn_nav_serialize(this[HANDLE], data, size), 'sn_nav_serialize');
	}

	/** 导航网格要比人群活得久。 */
	createCrowd(maxAgents: number, maxAgentRadius: number): Crowd {
		const n = this.native;
		return new Crowd(n, n.check(n.module._sn_crowd_create(this[HANDLE], maxAgents, maxAgentRadius), 'sn_crowd_create'), maxAgents);
	}

	protected override release(): void {
		this.native.module._sn_nav_destroy(this[HANDLE]);
	}
}

export class Crowd extends NativeObject<StrideNative> {
	readonly maxAgents: number;
	readonly #positions: Float32Array;

	/** @internal */
	constructor(native: StrideNative, handle: number, maxAgents: number) {
		super(native, handle);
		this.maxAgents = maxAgents;
		this.#positions = native.f32(native.module._sn_crowd_positions(handle), maxAgents * 3);
	}

	addAgent(position: Vec3, { radius = 0.5, height = 2, maxSpeed = 3.5, maxAcceleration = 8 }: AgentOptions = {}): AgentId {
		const n = this.native;
		return n.withArrays([new Float32Array(position)], (p) =>
			n.check(n.module._sn_crowd_add_agent(this[HANDLE], p, radius, height, maxSpeed, maxAcceleration), 'sn_crowd_add_agent')) as AgentId;
	}

	removeAgent(agent: AgentId): void {
		this.native.module._sn_crowd_remove_agent(this[HANDLE], agent);
	}

	requestMove(agent: AgentId, target: Vec3): void {
		const n = this.native;
		n.withArrays([new Float32Array(target)], (t) => n.check(n.module._sn_crowd_request_move(this[HANDLE], agent, t), 'sn_crowd_request_move'));
	}

	/** 移动代理并刷新 positions()。 */
	update(deltaTime: number): void {
		this.native.module._sn_crowd_update(this[HANDLE], deltaTime);
	}

	/**
	 * 每个槽位的 xyz（空槽位是 NaN），按 AgentId 排：原生内存上的视图，update() 原地改写，始终是同一个对象。
	 * 人群销毁后失效。
	 */
	positions(): Float32Array {
		return this.#positions;
	}

	protected override release(): void {
		this.native.module._sn_crowd_destroy(this[HANDLE]);
	}
}

export class Skeleton extends NativeObject<StrideNative> {
	readonly jointCount: number;

	/** @internal */
	constructor(native: StrideNative, handle: number) {
		super(native, handle);
		this.jointCount = native.module._sn_anim_skeleton_joint_count(handle);
	}

	/** 骨架要比它的实例活得久。 */
	createInstance(): AnimationInstance {
		const n = this.native;
		return new AnimationInstance(n, n.check(n.module._sn_anim_instance_create(this[HANDLE]), 'sn_anim_instance_create'), this.jointCount);
	}

	protected override release(): void {
		this.native.module._sn_anim_destroy_skeleton(this[HANDLE]);
	}
}

export class Animation extends NativeObject<StrideNative> {
	/** 秒。 */
	readonly duration: number;

	/** @internal */
	constructor(native: StrideNative, handle: number) {
		super(native, handle);
		this.duration = native.module._sn_anim_animation_duration(handle);
	}

	protected override release(): void {
		this.native.module._sn_anim_destroy_animation(this[HANDLE]);
	}
}

export class AnimationInstance extends NativeObject<StrideNative> {
	readonly jointCount: number;

	/** @internal */
	constructor(native: StrideNative, handle: number, jointCount: number) {
		super(native, handle);
		this.jointCount = jointCount;
	}

	/** 模型空间的关节矩阵（每个是列主序 4x4），原生内存上的视图，不拷贝，可以直接交给 GPUQueue.writeBuffer。实例销毁后失效。 */
	modelMatrices(): Float32Array {
		return this.native.f32(this.native.module._sn_anim_instance_model_matrices(this[HANDLE]), this.jointCount * 16);
	}

	protected override release(): void {
		this.native.module._sn_anim_instance_destroy(this[HANDLE]);
	}
}

/**
 * 一帧要算的动画：每个实例一段后缀程序（push 采样剪辑，blend、add、subtract 合并栈顶两个姿势，同 Stride 的
 * AnimationOperation），可选地再算蒙皮矩阵。请求直接写在原生内存里，每帧 clear() 后重填，不分配 JS 对象。
 *
 *   batch.clear();
 *   batch.instance(a).push(idle, t0).push(walk, t1).blend(0.3).skin(skin, palette);
 *   batch.evaluate();
 */
export class AnimationBatch implements Disposable {
	readonly #native: StrideNative;
	#ops = new NativeBuffer(16);
	#requests = new NativeBuffer(16);
	#skins = new NativeBuffer(16);
	#opCount = 0;
	#requestCount = 0;
	#skinCount = 0;
	/** 当前实例的请求下标。 */
	#current = -1;

	constructor(native: StrideNative) {
		this.#native = native;
	}

	clear(): this {
		this.#opCount = this.#requestCount = this.#skinCount = 0;
		this.#current = -1;
		return this;
	}

	/** 开始一个实例的程序；之后的 push、blend 等都属于它。没有操作的实例求出静止姿势。 */
	instance(instance: AnimationInstance): this {
		const at = this.#requests.ensure(this.#native, this.#requestCount + 1) + this.#requestCount * 16;
		const view = this.#native.i32(at, 4);
		view[0] = instance[HANDLE];
		view[1] = this.#opCount;
		view[2] = 0;
		view[3] = 0;
		this.#current = this.#requestCount++;
		return this;
	}

	/** 把剪辑在 time 秒处的姿势压栈（time 截到 [0, 时长]）。 */
	push(animation: Animation, time: number): this {
		return this.#op(ANIM_OPS.push, animation[HANDLE], time, 0);
	}

	/** 弹出 a、b，压入 lerp(a, b, factor)（旋转是 slerp）。 */
	blend(factor: number): this {
		return this.#op(ANIM_OPS.blend, 0, 0, factor);
	}

	/** 弹出 a、b，压入从 a 往 a + b 插值 factor 的结果（b 是叠加动画）。 */
	add(factor: number): this {
		return this.#op(ANIM_OPS.add, 0, 0, factor);
	}

	/** 弹出 a、b，压入从 a 往 a − b 插值 factor 的结果。 */
	subtract(factor: number): this {
		return this.#op(ANIM_OPS.subtract, 0, 0, factor);
	}

	/** 求值之后，把当前实例的蒙皮矩阵写进 palette。 */
	skin(skin: Skin, palette: SkinPalette): this {
		if (this.#current < 0) throw new NativeError('AnimationBatch.skin: call instance() first', -1);
		if (palette.jointCount !== skin.jointCount) throw new NativeError(`AnimationBatch.skin: the palette holds ${palette.jointCount} joints, the skin has ${skin.jointCount}`, -1);
		const at = this.#skins.ensure(this.#native, this.#skinCount + 1) + this.#skinCount * 16;
		const view = this.#native.i32(at, 4);
		view[0] = this.#native.i32(this.#requests.address + this.#current * 16, 1)[0]!;
		view[1] = skin[HANDLE];
		view[2] = palette.address;
		view[3] = 0;
		this.#skinCount++;
		return this;
	}

	/** 并行求所有实例的姿势（模型空间的关节矩阵），再算所有蒙皮矩阵。 */
	evaluate(): void {
		const m = this.#native.module;
		if (this.#requestCount > 0) this.#native.check(m._sn_anim_evaluate_batch(this.#ops.address, this.#requests.address, this.#requestCount), 'sn_anim_evaluate_batch');
		if (this.#skinCount > 0) this.#native.check(m._sn_anim_skin_batch(this.#skins.address, this.#skinCount), 'sn_anim_skin_batch');
	}

	[Symbol.dispose](): void {
		for (const buffer of [this.#ops, this.#requests, this.#skins]) buffer.free(this.#native);
	}

	#op(type: number, animation: number, time: number, factor: number): this {
		if (this.#current < 0) throw new NativeError('AnimationBatch: call instance() before adding operations', -1);
		const at = this.#ops.ensure(this.#native, this.#opCount + 1) + this.#opCount * 16;
		const ints = this.#native.i32(at, 2);
		ints[0] = type;
		ints[1] = animation;
		const floats = this.#native.f32(at + 8, 2);
		floats[0] = time;
		floats[1] = factor;
		this.#opCount++;
		// 当前实例的 op_count
		this.#native.i32(this.#requests.address + this.#current * 16 + 8, 1)[0]! += 1;
		return this;
	}
}

/** sn_anim_op.type。 */
const ANIM_OPS = { push: 0, blend: 1, add: 2, subtract: 3 } as const;

/** 原生内存里按元素大小增长的数组（AnimationBatch 的几张请求表）。 */
class NativeBuffer {
	readonly #itemSize: number;
	#address = 0;
	#capacity = 0;

	constructor(itemSize: number) {
		this.#itemSize = itemSize;
	}

	get address(): number {
		return this.#address;
	}

	/** 至少能放 count 个元素（不够时翻倍，保留已有内容），返回地址。 */
	ensure(native: StrideNative, count: number): number {
		if (count <= this.#capacity) return this.#address;
		const capacity = Math.max(count, this.#capacity * 2, 16);
		const address = native.alloc(capacity * this.#itemSize);
		if (this.#address) {
			new Uint8Array(native.buffer, address, this.#capacity * this.#itemSize).set(new Uint8Array(native.buffer, this.#address, this.#capacity * this.#itemSize));
			native.free(this.#address);
		}
		this.#address = address;
		this.#capacity = capacity;
		return address;
	}

	free(native: StrideNative): void {
		if (this.#address) native.free(this.#address);
		this.#address = this.#capacity = 0;
	}
}

export class Skin extends NativeObject<StrideNative> {
	readonly jointCount: number;

	/** @internal */
	constructor(native: StrideNative, handle: number, jointCount: number) {
		super(native, handle);
		this.jointCount = jointCount;
	}

	/** 放这个蒙皮的蒙皮矩阵的原生缓冲（每个画出来的角色一个）。 */
	createPalette(): SkinPalette {
		return new SkinPalette(this.native, this.jointCount);
	}

	protected override release(): void {
		this.native.module._sn_anim_skin_destroy(this[HANDLE]);
	}
}

/** 蒙皮矩阵：每个关节一个列主序 4x4，在原生内存里，AnimationBatch.evaluate 写，渲染器直接上传。 */
export class SkinPalette implements Disposable {
	readonly jointCount: number;
	readonly #native: StrideNative;
	#address: number;

	/** @internal */
	constructor(native: StrideNative, jointCount: number) {
		this.#native = native;
		this.jointCount = jointCount;
		this.#address = native.alloc(jointCount * 64);
		this.matrices().fill(0);
	}

	/** @internal */
	get address(): number {
		return this.#address;
	}

	/** 原生内存上的视图，不拷贝（内存增长后要重新取）。 */
	matrices(): Float32Array {
		if (!this.#address) throw new NativeError('SkinPalette was destroyed', -2);
		return this.#native.f32(this.#address, this.jointCount * 16);
	}

	[Symbol.dispose](): void {
		if (this.#address) this.#native.free(this.#address);
		this.#address = 0;
	}
}
