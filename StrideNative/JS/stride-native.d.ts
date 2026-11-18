// Types for stride-native.mjs: the runtime module (browser and Node).
// The page must be cross-origin isolated (COOP: same-origin, COEP: require-corp).

export class NativeError extends Error {
	/** SN_ERROR_* code (negative). */
	readonly code: number;
}

export interface ModuleOptions {
	/** Job-system worker threads; defaults to navigator.hardwareConcurrency - 1 (at least 1). */
	workerThreads?: number;
	/** Emscripten locateFile hook, to serve the .wasm from another path. */
	locateFile?: (path: string, prefix: string) => string;
}

export function defaultWorkerThreads(): number;
export function createStrideNative(options?: ModuleOptions): Promise<StrideNative>;

export const MeshoptMode: Readonly<{ attributes: 0; triangles: 1; indices: 2 }>;
export const MeshoptFilter: Readonly<{ none: 0; octahedral: 1; quaternion: 2; exponential: 3; color: 4 }>;
export const TextureTarget: Readonly<{ rgba8: 0; bc7: 1; astc4x4: 2; etc2rgba8: 3 }>;

type Vec3 = ArrayLike<number>;

export interface MeshoptRequest {
	source: Uint8Array;
	count: number;
	stride: number;
	mode?: number;
	filter?: number;
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
	/** Bytes per 4x4 block (compressed targets) or per pixel (rgba8). */
	unitSize: number;
	/** Mip levels, largest first. */
	levels: Uint8Array[];
}

/** Recast settings; omitted fields use the Recast demo defaults for a human-sized agent. */
export interface NavConfig {
	cellSize?: number;
	cellHeight?: number;
	agentHeight?: number;
	agentRadius?: number;
	agentMaxClimb?: number;
	agentMaxSlopeDegrees?: number;
	/** In cells; tiles are built in parallel. */
	tileSize?: number;
	maxEdgeLength?: number;
	maxSimplificationError?: number;
	minRegionSize?: number;
	mergeRegionSize?: number;
	detailSampleDistance?: number;
	detailSampleMaxError?: number;
}

export interface AnimationSampleRequest {
	instance: AnimationInstance;
	animation: Animation;
	/** Normalized time in [0, 1]. */
	ratio: number;
}

export class StrideNative {
	readonly workerThreadCount: number;
	/** The module's shared memory (it grows; take views per use). */
	readonly buffer: SharedArrayBuffer;
	shutdown(): void;

	createPhysicsWorld(maxBodies?: number): PhysicsWorld;

	decodeMeshopt(source: Uint8Array, count: number, stride: number, mode?: number, filter?: number): Uint8Array;
	/** Decodes in parallel on the job system. */
	decodeMeshoptBatch(requests: MeshoptRequest[]): Promise<Uint8Array[]>;

	ktx2Info(bytes: Uint8Array): Ktx2Info;
	/** Transcodes all mip levels in parallel. */
	transcodeKtx2(bytes: Uint8Array, target?: number): Promise<TranscodedTexture>;

	/** Builds a tiled navmesh from a triangle soup; tiles in parallel. */
	buildNavMesh(vertices: Float32Array, triangles: Int32Array, config?: NavConfig): Promise<NavMesh>;
	loadNavMesh(bytes: Uint8Array): NavMesh;

	loadSkeleton(ozzArchive: Uint8Array): Skeleton;
	loadAnimation(ozzArchive: Uint8Array): Animation;
	/** Samples all requests in parallel; returns when every instance's matrices are updated. */
	sampleAnimations(requests: AnimationSampleRequest[]): void;
}

export interface BoxDesc {
	position: Vec3;
	halfExtent: Vec3;
	/** Defaults to 'dynamic'. */
	motion?: 'dynamic' | 'static';
}

export class PhysicsWorld {
	readonly handle: number;
	readonly maxBodies: number;
	readonly bodyCount: number;
	/** Returns the body index (creation order). Allocates nothing: the descriptor goes through a reused native buffer. */
	addBox(box: BoxDesc): number;
	/** Adds the boxes in one native call; Jolt inserts them into the broadphase as a batch. Returns the first body index. */
	addBoxes(boxes: readonly BoxDesc[]): number;
	/** Steps on the shared job system (the calling thread runs jobs too while it waits), then refreshes positions(). */
	step(deltaTime: number, collisionSteps?: number): void;
	/**
	 * xyz per body in creation order: a view of native memory that step() rewrites in place, so no copy and
	 * the same object until bodies are added. Valid until the world is destroyed.
	 */
	positions(): Float32Array;
	destroy(): void;
}

export class NavMesh {
	readonly handle: number;
	/** Straight path corners (xyz). Throws NativeError when there is no path. */
	findPath(start: Vec3, end: Vec3, searchExtents?: Vec3, maxPoints?: number): Float32Array;
	serialize(): Uint8Array;
	/** The navmesh must outlive the crowd. */
	createCrowd(maxAgents: number, maxAgentRadius: number): Crowd;
	destroy(): void;
}

export interface AgentOptions {
	radius?: number;
	height?: number;
	maxSpeed?: number;
	maxAcceleration?: number;
}

export class Crowd {
	readonly handle: number;
	/** Returns the agent index. */
	addAgent(position: Vec3, options?: AgentOptions): number;
	removeAgent(agent: number): void;
	requestMove(agent: number, target: Vec3): void;
	/** Moves the agents and refreshes positions(). */
	update(deltaTime: number): void;
	/**
	 * xyz per agent slot (NaN for empty slots): a view of native memory that update() rewrites in place,
	 * always the same object. Valid until the crowd is destroyed.
	 */
	positions(): Float32Array;
	destroy(): void;
}

export class Skeleton {
	readonly handle: number;
	readonly jointCount: number;
	/** The skeleton must outlive its instances. */
	createInstance(): AnimationInstance;
	destroy(): void;
}

export class Animation {
	readonly handle: number;
	/** Seconds. */
	readonly duration: number;
	destroy(): void;
}

export class AnimationInstance {
	readonly handle: number;
	readonly jointCount: number;
	/** Model-space joint matrices (column-major 4x4), a view of native memory: no copy. */
	modelMatrices(): Float32Array;
	destroy(): void;
}
