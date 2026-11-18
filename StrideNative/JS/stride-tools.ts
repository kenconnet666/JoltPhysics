// stride-tools 资源管线模块的 JS API（Node）。耗时的操作返回原生任务背后的 Promise，可以同时处理多个资源：
// 一起发起、一起 await，它们共用模块的任务系统。
import createStrideToolsCore, { type ToolsExport } from './stride-tools-core.mjs';
import { NAV_CONFIG_SIZE, NativeModule, NativeObject, HANDLE, enumCode, instantiate, writeNavConfig, type ModuleOptions, type NavConfig } from './native-module.ts';

export { NativeError, defaultWorkerThreads } from './native-module.ts';
export type { ModuleOptions, NativeErrorCode, NavConfig } from './native-module.ts';

export type SceneFormat = 'auto' | 'gltf' | 'fbx';
export type BasisMode = 'uastc' | 'etc1s';

const SCENE_FORMATS: Record<SceneFormat, number> = { auto: 0, gltf: 1, fbx: 2 };
const BASIS_MODES: Record<BasisMode, number> = { uastc: 0, etc1s: 1 };

export interface MeshData {
	positions: Float32Array;
	normals: Float32Array | null;
	uvs: Float32Array | null;
	indices: Uint32Array;
}

export interface UvUnwrap {
	/** 每个新顶点对应的输入顶点下标。 */
	xref: Uint32Array;
	/** [0, 1] 内的 UV，每个新顶点两个。 */
	uvs: Float32Array;
	indices: Uint32Array;
	width: number;
	height: number;
}

export interface ConvexHull {
	vertices: Float32Array;
	triangles: Uint32Array;
}

export interface ConvexOptions {
	maxHulls?: number;
	voxelResolution?: number;
	maxVerticesPerHull?: number;
}

export interface BasisOptions {
	/** 默认 'uastc'。 */
	mode?: BasisMode;
	srgb?: boolean;
	mipmaps?: boolean;
	quality?: number;
	effort?: number;
}

export interface Glyph {
	codepoint: number;
	width: number;
	height: number;
	bearingX: number;
	bearingY: number;
	advance: number;
	/** RGB8，行从上到下；没有轮廓的字形是空的。 */
	pixels: Uint8Array;
}

export interface JointDescription {
	/** 根是 -1；父关节必须排在前面。 */
	parent?: number;
	translation?: readonly [number, number, number];
	/** 四元数 xyzw。 */
	rotation?: readonly [number, number, number, number];
	scale?: readonly [number, number, number];
	name?: string;
}

export interface JointTrack {
	/** [时间, x, y, z] */
	translations?: readonly (readonly number[])[];
	/** [时间, x, y, z, w] */
	rotations?: readonly (readonly number[])[];
	/** [时间, x, y, z] */
	scales?: readonly (readonly number[])[];
}

export async function createStrideTools(options: ModuleOptions = {}): Promise<StrideTools> {
	return new StrideTools(await instantiate(createStrideToolsCore, options));
}

export class StrideTools extends NativeModule<ToolsExport> {
	/** 导入 glTF/GLB（内嵌缓冲）或 FBX（转换成右手、Y 向上、米）。 */
	loadScene(bytes: Uint8Array, format: SceneFormat = 'auto'): Promise<Scene> {
		const code = enumCode(SCENE_FORMATS, format, 'format');
		return this.withArraysAsync([bytes], (data) =>
			this.awaitTask(this.module._st_scene_load(data, bytes.byteLength, code), 'st_scene_load',
				(task) => new Scene(this, task.result(0), task.result(1))));
	}

	/** MikkTSpace 切线：每个索引（角）一个 xyzw。 */
	generateTangents({ positions, normals, uvs, indices }: { positions: Float32Array; normals: Float32Array; uvs: Float32Array; indices: Uint32Array }): Float32Array {
		return this.withArrays([positions, normals, uvs, indices], (p, n, u, i) => this.withScratch(indices.length * 16, (out) => {
			this.check(this.module._st_generate_tangents(p, n, u, i, indices.length, out), 'st_generate_tangents');
			return this.copyOut(Float32Array, out, indices.length * 4);
		}));
	}

	/** 顶点缓存加 overdraw 优化；返回重排后的索引。 */
	optimizeMesh(indices: Uint32Array, positions: Float32Array): Uint32Array {
		return this.withArrays([indices, positions], (i, p) => {
			this.check(this.module._st_meshopt_optimize(i, indices.length, p, positions.length / 3, 12), 'st_meshopt_optimize');
			return this.copyOut(Uint32Array, i, indices.length);
		});
	}

	simplifyMesh(indices: Uint32Array, positions: Float32Array, targetIndexCount: number, targetError = 0.01): { indices: Uint32Array; error: number } {
		return this.withArrays([indices, positions], (i, p) => this.withScratch(indices.length * 4 + 4, (out) => {
			const errorPtr = out + indices.length * 4;
			const count = this.check(this.module._st_meshopt_simplify(out, i, indices.length, p, positions.length / 3, 12, targetIndexCount, targetError, errorPtr), 'st_meshopt_simplify');
			return { indices: this.copyOut(Uint32Array, out, count), error: this.f32(errorPtr, 1)[0]! };
		}));
	}

	/** meshopt 顶点编码（glTF EXT_meshopt_compression 的 "ATTRIBUTES"）。 */
	encodeVertices(vertices: Uint8Array, vertexSize: number): Uint8Array {
		const count = vertices.byteLength / vertexSize;
		const capacity = this.module._st_meshopt_encode_vertex_bound(count, vertexSize);
		return this.withArrays([vertices], (v) => this.withScratch(capacity, (out) => {
			const size = this.module._st_meshopt_encode_vertices(out, capacity, v, count, vertexSize);
			return this.copyOut(Uint8Array, out, this.check(size > 0 ? size : -3, 'st_meshopt_encode_vertices'));
		}));
	}

	/** meshopt 索引编码（glTF EXT_meshopt_compression 的 "TRIANGLES"）。 */
	encodeIndices(indices: Uint32Array, vertexCount: number): Uint8Array {
		const capacity = this.module._st_meshopt_encode_index_bound(indices.length, vertexCount);
		return this.withArrays([indices], (i) => this.withScratch(capacity, (out) => {
			const size = this.module._st_meshopt_encode_indices(out, capacity, i, indices.length);
			return this.copyOut(Uint8Array, out, this.check(size > 0 ? size : -3, 'st_meshopt_encode_indices'));
		}));
	}

	/** 光照贴图 UV。xref 把新顶点映射回输入顶点。 */
	unwrapUvs({ positions, normals, indices }: { positions: Float32Array; normals?: Float32Array | null; indices: Uint32Array }, texelsPerUnit = 0): Promise<UvUnwrap> {
		return this.withArraysAsync([positions, normals, indices], (p, n, i) =>
			this.awaitTask(this.module._st_uv_unwrap(p, n, positions.length / 3, i, indices.length, texelsPerUnit), 'st_uv_unwrap', (task) => ({
				xref: task.output(Uint32Array, 0),
				uvs: task.output(Float32Array, 1),
				indices: task.output(Uint32Array, 2),
				width: task.result(0),
				height: task.result(1),
			})));
	}

	/** 给物理用的凸分解。 */
	convexDecompose({ positions, indices }: { positions: Float32Array; indices: Uint32Array }, { maxHulls = 16, voxelResolution = 100000, maxVerticesPerHull = 64 }: ConvexOptions = {}): Promise<ConvexHull[]> {
		return this.withArraysAsync([positions, indices], (p, i) =>
			this.awaitTask(this.module._st_convex_decompose(p, positions.length / 3, i, indices.length, maxHulls, voxelResolution, maxVerticesPerHull), 'st_convex_decompose', (task) => {
				const counts = task.output(Int32Array, 0);
				const points = task.output(Float32Array, 1);
				const triangles = task.output(Uint32Array, 2);
				const hulls: ConvexHull[] = [];
				for (let h = 0, point = 0, triangle = 0; h < task.result(0); ++h) {
					const vertexCount = counts[h * 2]!, triangleCount = counts[h * 2 + 1]!;
					hulls.push({
						vertices: points.slice(point * 3, (point + vertexCount) * 3),
						triangles: triangles.slice(triangle * 3, (triangle + triangleCount) * 3),
					});
					point += vertexCount;
					triangle += triangleCount;
				}
				return hulls;
			}));
	}

	/** 把 RGBA8 像素编码成 KTX2 文件。 */
	encodeBasis(rgba: Uint8Array, width: number, height: number, { mode = 'uastc', srgb = true, mipmaps = true, quality = 75, effort = 2 }: BasisOptions = {}): Promise<Uint8Array> {
		const code = enumCode(BASIS_MODES, mode, 'BasisOptions.mode');
		const flags = (srgb ? 1 : 0) | (mipmaps ? 2 : 0);
		return this.withArraysAsync([rgba], (pixels) =>
			this.awaitTask(this.module._st_basis_encode(pixels, width, height, code, flags, quality, effort), 'st_basis_encode',
				(task) => task.output(Uint8Array, 0)));
	}

	/** 并行生成 MSDF 字形。 */
	msdfGlyphs(font: Uint8Array, codepoints: Iterable<number>, pixelSize = 32, distanceRange = 4): Promise<Glyph[]> {
		const points = new Uint32Array([...codepoints]);
		return this.withArraysAsync([font, points], (f, c) =>
			this.awaitTask(this.module._st_font_msdf(f, font.byteLength, c, points.length, pixelSize, distanceRange), 'st_font_msdf', (task) => {
				const metrics = new DataView(task.output(Uint8Array, 0).buffer);
				return Array.from(points, (codepoint, i) => ({
					codepoint,
					width: metrics.getUint32(i * 24 + 4, true),
					height: metrics.getUint32(i * 24 + 8, true),
					bearingX: metrics.getFloat32(i * 24 + 12, true),
					bearingY: metrics.getFloat32(i * 24 + 16, true),
					advance: metrics.getFloat32(i * 24 + 20, true),
					pixels: task.output(Uint8Array, i + 1),
				}));
			}));
	}

	/** 生成 ozz 骨架档案。 */
	buildSkeleton(joints: readonly JointDescription[]): Uint8Array {
		const JOINT_SIZE = 44;
		const table = new DataView(new ArrayBuffer(joints.length * JOINT_SIZE));
		joints.forEach((joint, i) => {
			const base = i * JOINT_SIZE;
			table.setInt32(base, joint.parent ?? -1, true);
			const floats = [...(joint.translation ?? [0, 0, 0]), ...(joint.rotation ?? [0, 0, 0, 1]), ...(joint.scale ?? [1, 1, 1])];
			floats.forEach((value, f) => table.setFloat32(base + 4 + f * 4, value, true));
		});
		const names = new TextEncoder().encode(joints.map((joint, i) => (joint.name ?? `joint${i}`) + '\0').join(''));
		return this.withArrays([new Uint8Array(table.buffer), names], (j, n) =>
			this.readOwnedBuffer((data, size) => this.module._st_anim_build_skeleton(j, joints.length, n, data, size), 'st_anim_build_skeleton'));
	}

	/** 生成 ozz 动画档案：每个关节一条轨道，时间以秒计。 */
	buildAnimation(duration: number, tracks: readonly JointTrack[]): Uint8Array {
		const counts = new Uint32Array(tracks.length * 3);
		const keys: number[] = [];
		tracks.forEach((track, j) => {
			[track.translations, track.rotations, track.scales].forEach((list = [], k) => {
				counts[j * 3 + k] = list.length;
				for (const key of list) keys.push(...key);
			});
		});
		return this.withArrays([counts, new Float32Array(keys)], (c, k) =>
			this.readOwnedBuffer((data, size) => this.module._st_anim_build_animation(duration, tracks.length, c, k, data, size), 'st_anim_build_animation'));
	}

	/** 烘焙给 StrideNative.loadNavMesh 用的导航网格，各 tile 并行。 */
	bakeNavMesh(vertices: Float32Array, triangles: Int32Array, config: NavConfig = {}): Promise<Uint8Array> {
		return this.withArraysAsync([vertices, triangles], (v, t) => this.withScratch(NAV_CONFIG_SIZE, (cfg) => {
			writeNavConfig(this, cfg, config);
			return this.awaitTask(this.module._st_nav_bake(v, vertices.length / 3, t, triangles.length / 3, cfg), 'st_nav_bake',
				(task) => task.output(Uint8Array, 0));
		}));
	}
}

export class Scene extends NativeObject<StrideTools> {
	readonly meshCount: number;

	/** @internal */
	constructor(tools: StrideTools, handle: number, meshCount: number) {
		super(tools, handle);
		this.meshCount = meshCount;
	}

	mesh(index: number): MeshData {
		const t = this.native;
		return t.withScratch(16, (infoPtr) => {
			t.check(t.module._st_scene_mesh_info(this[HANDLE], index, infoPtr), 'st_scene_mesh_info');
			const info = t.u32(infoPtr, 4);
			const vertexCount = info[0]!, indexCount = info[1]!, hasNormals = !!info[2], hasUvs = !!info[3];
			const positions = t.alloc(vertexCount * 12);
			const normals = hasNormals ? t.alloc(vertexCount * 12) : 0;
			const uvs = hasUvs ? t.alloc(vertexCount * 8) : 0;
			const indices = t.alloc(indexCount * 4);
			try {
				t.check(t.module._st_scene_mesh_copy(this[HANDLE], index, positions, normals, uvs, indices), 'st_scene_mesh_copy');
				return {
					positions: t.copyOut(Float32Array, positions, vertexCount * 3),
					normals: hasNormals ? t.copyOut(Float32Array, normals, vertexCount * 3) : null,
					uvs: hasUvs ? t.copyOut(Float32Array, uvs, vertexCount * 2) : null,
					indices: t.copyOut(Uint32Array, indices, indexCount),
				};
			} finally {
				for (const ptr of [positions, normals, uvs, indices]) if (ptr) t.free(ptr);
			}
		});
	}

	protected override release(): void {
		this.native.module._st_scene_destroy(this[HANDLE]);
	}
}
