// stride-native 运行时模块的 JS API（浏览器和 Node）。页面必须跨源隔离（COOP: same-origin，COEP: require-corp）。
import createStrideNativeCore, { type RuntimeExport } from './stride-native-core.mjs';
import {
	HANDLE, NAV_CONFIG_SIZE, NativeModule, NativeObject, enumCode, instantiate, writeNavConfig,
	type Branded, type ModuleOptions, type NavConfig,
} from './native-module.ts';

export { NativeError, defaultWorkerThreads } from './native-module.ts';
export type { ModuleOptions, NativeErrorCode, NavConfig } from './native-module.ts';

/** xyz；数组、元组和 Vector3 之类的 ArrayLike 都行。 */
export type Vec3 = ArrayLike<number>;

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

export interface BoxDesc {
	/** 初始位置，默认原点；绑定了 transform 时不用它。 */
	position?: Vec3;
	halfExtent: Vec3;
	/** 默认 'dynamic'。 */
	motion?: 'dynamic' | 'static';
	/**
	 * 绑定的变换槽位（世界要用 createPhysicsWorld(n, { transforms }) 创建）。初始位姿取槽位当前的世界位姿（缩放不作用到形状上），
	 * 动态物体每步结束把位姿写回槽位的本地位置和旋转，有父节点时换算到父节点空间。物体存在期间槽位不能释放。
	 */
	transform?: TransformSlot;
}

export interface PhysicsWorldOptions {
	/** 物体可以绑定的变换存储；它要比世界活得久。 */
	transforms?: TransformStore;
}

export interface AgentOptions {
	radius?: number;
	height?: number;
	maxSpeed?: number;
	maxAcceleration?: number;
}

export interface AnimationSampleRequest {
	instance: AnimationInstance;
	animation: Animation;
	/** 归一化时间，[0, 1]。 */
	ratio: number;
}

/** 加载模块并启动任务系统。 */
export async function createStrideNative(options: ModuleOptions = {}): Promise<StrideNative> {
	return new StrideNative(await instantiate(createStrideNativeCore, options));
}

export class StrideNative extends NativeModule<RuntimeExport> {
	/** 能放 capacity 个槽位的变换存储。各列一次分配好，视图一直有效。 */
	createTransforms(capacity: number): TransformStore {
		return new TransformStore(this, this.check(this.module._sn_transforms_create(capacity), 'sn_transforms_create'));
	}

	createPhysicsWorld(maxBodies = 1024, { transforms }: PhysicsWorldOptions = {}): PhysicsWorld {
		const handle = this.module._sn_physics_create(maxBodies, transforms ? transforms[HANDLE] : 0);
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

	ktx2Info(bytes: Uint8Array): Ktx2Info {
		return this.withArrays([bytes], (data) => this.withScratch(32, (info) => {
			this.check(this.module._sn_ktx2_get_info(data, bytes.byteLength, info), 'sn_ktx2_get_info');
			const v = this.i32(info, 8);
			return { width: v[0]!, height: v[1]!, levels: v[2]!, layers: v[3]!, faces: v[4]!, hasAlpha: !!v[5], isSrgb: !!v[6], isHdr: !!v[7] };
		}));
	}

	/** 并行转码所有 mip 级。 */
	transcodeKtx2(bytes: Uint8Array, target: TextureTarget = 'rgba8'): Promise<TranscodedTexture> {
		const code = enumCode(TEXTURE_TARGETS, target, 'target');
		return this.withArraysAsync([bytes], (data) =>
			this.awaitTask(this.module._sn_ktx2_transcode(data, bytes.byteLength, code), 'sn_ktx2_transcode', (task) => ({
				width: task.result(0),
				height: task.result(1),
				unitSize: task.result(3),
				levels: Array.from({ length: task.result(2) }, (_, level) => task.output(Uint8Array, level)),
			})));
	}

	/** 从三角形汤构建分块导航网格，各 tile 并行。 */
	buildNavMesh(vertices: Float32Array, triangles: Int32Array, config: NavConfig = {}): Promise<NavMesh> {
		return this.withArraysAsync([vertices, triangles], (v, t) => this.withScratch(NAV_CONFIG_SIZE, (cfg) => {
			writeNavConfig(this, cfg, config);
			return this.awaitTask(this.module._sn_nav_build(v, vertices.length / 3, t, triangles.length / 3, cfg), 'sn_nav_build',
				(task) => new NavMesh(this, task.result(0)));
		}));
	}

	/** 加载 NavMesh.serialize 或 stride-tools 烘焙出的导航网格。 */
	loadNavMesh(bytes: Uint8Array): NavMesh {
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

	/** 并行采样所有请求；返回时每个实例的矩阵都已更新。 */
	sampleAnimations(requests: readonly AnimationSampleRequest[]): void {
		const REQUEST_SIZE = 16;
		this.withScratch(Math.max(1, requests.length) * REQUEST_SIZE, (table) => {
			requests.forEach(({ instance, animation, ratio }, i) => {
				const view = this.view(table + i * REQUEST_SIZE, REQUEST_SIZE);
				view.setInt32(0, instance[HANDLE], true);
				view.setInt32(4, animation[HANDLE], true);
				view.setFloat32(8, ratio, true);
			});
			this.check(this.module._sn_anim_sample_batch(table, requests.length), 'sn_anim_sample_batch');
		});
	}
}

/** sn_box：position（3 个 float）、half extent（3 个 float）、flags（uint32）、transform（int32）。 */
const BOX_WORDS = 8;
const BODY_DYNAMIC = 1;

/** 用 StrideNative.createPhysicsWorld 创建。 */
export class PhysicsWorld extends NativeObject<StrideNative> {
	readonly maxBodies: number;
	#bodyCount = 0;
	readonly #positionsAddress: number;
	#positions: Float32Array;
	// addBox 复用的一个 sn_box，加一个刚体不分配内存
	readonly #scratch: number;
	readonly #scratchFloats: Float32Array;
	readonly #scratchWords: Uint32Array;

	/** @internal */
	constructor(native: StrideNative, handle: number, maxBodies: number) {
		super(native, handle);
		this.maxBodies = maxBodies;
		this.#positionsAddress = native.module._sn_physics_positions(handle);
		this.#positions = native.f32(this.#positionsAddress, 0);
		this.#scratch = native.alloc(BOX_WORDS * 4);
		this.#scratchFloats = native.f32(this.#scratch, BOX_WORDS);
		this.#scratchWords = native.u32(this.#scratch, BOX_WORDS);
	}

	get bodyCount(): number {
		return this.#bodyCount;
	}

	/** 加一个盒子。描述经由复用的原生缓冲传过去，不分配内存。 */
	addBox(box: BoxDesc): BodyId {
		writeBox(this.#scratchFloats, this.#scratchWords, 0, box);
		return this.#add(this.#scratch, 1)[0]!;
	}

	/** 一次原生调用加入所有盒子，Jolt 成批插进宽相。返回值和 boxes 一一对应。 */
	addBoxes(boxes: readonly BoxDesc[]): BodyId[] {
		if (boxes.length === 0) return [];
		const n = this.native;
		return n.withScratch(boxes.length * BOX_WORDS * 4, (table) => {
			const floats = n.f32(table, boxes.length * BOX_WORDS);
			const words = n.u32(table, boxes.length * BOX_WORDS);
			boxes.forEach((box, i) => writeBox(floats, words, i, box));
			return this.#add(table, boxes.length);
		});
	}

	#add(table: number, count: number): BodyId[] {
		const n = this.native;
		const first = n.check(n.module._sn_physics_add_boxes(this[HANDLE], table, count), 'sn_physics_add_boxes');
		this.#bodyCount = first + count;
		this.#positions = n.f32(this.#positionsAddress, this.#bodyCount * 3);
		return Array.from({ length: count }, (_, i) => (first + i) as BodyId);
	}

	/** 在共享任务系统上步进（当前线程等待时也跑任务），然后刷新 positions()。 */
	step(deltaTime: number, collisionSteps = 1): void {
		this.native.check(this.native.module._sn_physics_step(this[HANDLE], deltaTime, collisionSteps), 'sn_physics_step');
	}

	/**
	 * 每个刚体的 xyz，按 BodyId 排：原生内存上的视图，step() 原地改写，所以不拷贝，加刚体之前一直是同一个对象。
	 * 世界销毁后失效。
	 */
	positions(): Float32Array {
		return this.#positions;
	}

	protected override release(): void {
		this.native.module._sn_physics_destroy(this[HANDLE]);
		this.native.free(this.#scratch);
	}
}

/** 写 sn_box 表的第 i 个盒子；同一块内存分别按 float 和 uint32（flags、transform）看。 */
function writeBox(floats: Float32Array, words: Uint32Array, i: number, { position = ORIGIN, halfExtent, motion = 'dynamic', transform = 0 as TransformSlot }: BoxDesc): void {
	if (motion !== 'dynamic' && motion !== 'static') {
		throw new TypeError(`box ${i}: motion must be 'dynamic' or 'static', not '${String(motion)}'`);
	}
	const o = i * BOX_WORDS;
	floats[o] = position[0]!;
	floats[o + 1] = position[1]!;
	floats[o + 2] = position[2]!;
	floats[o + 3] = halfExtent[0]!;
	floats[o + 4] = halfExtent[1]!;
	floats[o + 5] = halfExtent[2]!;
	words[o + 6] = motion === 'dynamic' ? BODY_DYNAMIC : 0;
	words[o + 7] = transform;
}

const ORIGIN: Vec3 = [0, 0, 0];

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

	/** 按父节点在前的顺序算所有世界矩阵。层级变了才重排顺序，平时是一遍顺序扫描。 */
	update(): void {
		this.native.check(this.native.module._sn_transforms_update(this[HANDLE]), 'sn_transforms_update');
	}

	protected override release(): void {
		this.native.module._sn_transforms_destroy(this[HANDLE]);
		this.native.free(this.#scratch);
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
