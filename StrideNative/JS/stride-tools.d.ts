// Types for stride-tools.mjs: the asset pipeline module (Node).
// Promise-returning methods run as native tasks; start several and await them together to use
// every worker thread.
import type { ModuleOptions, NavConfig } from './stride-native';

export { NativeError, defaultWorkerThreads, ModuleOptions, NavConfig } from './stride-native';

export function createStrideTools(options?: ModuleOptions): Promise<StrideTools>;

export const SceneFormat: Readonly<{ auto: 0; gltf: 1; fbx: 2 }>;
export const BasisMode: Readonly<{ uastc: 0; etc1s: 1 }>;

export interface MeshData {
	positions: Float32Array;
	normals: Float32Array | null;
	uvs: Float32Array | null;
	indices: Uint32Array;
}

export interface UvUnwrap {
	/** Input vertex index of each new vertex. */
	xref: Uint32Array;
	/** UVs in [0, 1], two per new vertex. */
	uvs: Float32Array;
	indices: Uint32Array;
	width: number;
	height: number;
}

export interface ConvexHull {
	vertices: Float32Array;
	triangles: Uint32Array;
}

export interface Glyph {
	codepoint: number;
	width: number;
	height: number;
	bearingX: number;
	bearingY: number;
	advance: number;
	/** RGB8, rows top-down; empty for glyphs without outline. */
	pixels: Uint8Array;
}

export interface JointDescription {
	/** -1 for roots; parents must come first. */
	parent: number;
	translation?: [number, number, number];
	/** Quaternion xyzw. */
	rotation?: [number, number, number, number];
	scale?: [number, number, number];
	name?: string;
}

export interface JointTrack {
	/** [time, x, y, z] */
	translations?: number[][];
	/** [time, x, y, z, w] */
	rotations?: number[][];
	/** [time, x, y, z] */
	scales?: number[][];
}

export class StrideTools {
	readonly workerThreadCount: number;
	shutdown(): void;

	/** glTF/GLB with embedded buffers, or FBX (converted to right-handed Y-up, meters). */
	loadScene(bytes: Uint8Array, format?: number): Promise<Scene>;

	/** MikkTSpace; one xyzw per index (corner). */
	generateTangents(mesh: { positions: Float32Array; normals: Float32Array; uvs: Float32Array; indices: Uint32Array }): Float32Array;
	optimizeMesh(indices: Uint32Array, positions: Float32Array): Uint32Array;
	simplifyMesh(indices: Uint32Array, positions: Float32Array, targetIndexCount: number, targetError?: number): { indices: Uint32Array; error: number };
	encodeVertices(vertices: Uint8Array, vertexSize: number): Uint8Array;
	encodeIndices(indices: Uint32Array, vertexCount: number): Uint8Array;
	unwrapUvs(mesh: { positions: Float32Array; normals?: Float32Array | null; indices: Uint32Array }, texelsPerUnit?: number): Promise<UvUnwrap>;
	convexDecompose(mesh: { positions: Float32Array; indices: Uint32Array }, options?: { maxHulls?: number; voxelResolution?: number; maxVerticesPerHull?: number }): Promise<ConvexHull[]>;

	encodeBasis(rgba: Uint8Array, width: number, height: number, options?: { mode?: number; srgb?: boolean; mipmaps?: boolean; quality?: number; effort?: number }): Promise<Uint8Array>;
	msdfGlyphs(font: Uint8Array, codepoints: Iterable<number>, pixelSize?: number, distanceRange?: number): Promise<Glyph[]>;

	buildSkeleton(joints: JointDescription[]): Uint8Array;
	buildAnimation(duration: number, tracks: JointTrack[]): Uint8Array;

	bakeNavMesh(vertices: Float32Array, triangles: Int32Array, config?: NavConfig): Promise<Uint8Array>;
}

export class Scene {
	readonly handle: number;
	readonly meshCount: number;
	mesh(index: number): MeshData;
	destroy(): void;
}
