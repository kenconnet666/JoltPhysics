// stride-tools 资源管线模块的 JS API（Node）。耗时的操作返回原生任务背后的 Promise，可以同时处理多个资源：
// 一起发起、一起 await，它们共用模块的任务系统。
import createStrideToolsCore from './stride-tools-core.mjs';
import type { ToolsExport } from './stride-tools-core.mjs';
import { NAV_CONFIG_SIZE, NativeModule, NativeObject, HANDLE, enumCode, instantiate, writeNavConfig, type ModuleOptions, type NavConfig } from './native-module.ts';

export { NativeError, defaultWorkerThreads } from './native-module.ts';
export type { ModuleOptions, NativeErrorCode, NavConfig } from './native-module.ts';

export type SceneFormat = 'auto' | 'gltf' | 'fbx';
export type BasisMode = 'uastc' | 'etc1s';

const SCENE_FORMATS: Record<SceneFormat, number> = { auto: 0, gltf: 1, fbx: 2 };
/** ST_NO_MATERIAL */
const NO_MATERIAL = 0xffffffff;
const BASIS_MODES: Record<BasisMode, number> = { uastc: 0, etc1s: 1 };

export interface MeshData {
	/** 模型空间（节点的世界变换已经烘焙进去；蒙皮网格按绑定时的姿势），每个顶点 xyz。 */
	positions: Float32Array;
	normals: Float32Array | null;
	uvs: Float32Array | null;
	indices: Uint32Array;
	/** 在 Scene.materials 里的下标；没有材质时是 null。 */
	material: number | null;
	/** 挂着这个网格的节点（Scene.nodes 的下标）。 */
	node: number;
	/** 蒙皮网格的蒙皮；别的网格是 null。 */
	skin: MeshSkin | null;
}

export interface MeshSkin {
	/** 影响这个网格的关节，Scene.nodes 的下标。 */
	jointNodes: Uint32Array;
	/** 每个关节一个列主序 4x4：烘焙后的顶点到关节空间（绑定时）。关节当前的节点到模型矩阵乘它就是蒙皮矩阵。 */
	inverseBinds: Float32Array;
	/** 每个顶点 4 个，jointNodes 的下标，按权重从大到小。 */
	joints: Uint16Array;
	/** 每个顶点 4 个，和为 1，没用的是 0。 */
	weights: Float32Array;
}

/** 场景的节点（静止姿势，文件里的默认姿势）。按深度优先排，父节点在前；FBX 的根节点叫 RootNode。 */
export interface SceneNode {
	name: string;
	/** 根是 -1。 */
	parent: number;
	translation: [number, number, number];
	/** 四元数 xyzw。 */
	rotation: [number, number, number, number];
	scale: [number, number, number];
	/** 列主序的节点到模型矩阵。 */
	world: Float32Array;
}

/** 烘焙好的动画：每个节点（按 Scene.nodes 的顺序）一条轨道，可以直接交给 buildAnimation。 */
export interface BakedAnimation {
	duration: number;
	tracks: JointTrack[];
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

export interface DecodedImage {
	width: number;
	height: number;
	/** 文件里的通道数（1 到 4）；pixels 总是 RGBA8。 */
	channels: number;
	/** RGBA8，行从上到下。 */
	pixels: Uint8Array;
}

/** RGBA32F 的立方体贴图：面的顺序 +X -X +Y -Y +Z -Z，每面的行从上到下（D3D/WebGPU 的布局）。 */
export interface CubemapData {
	/** 一个面的边长（2 的幂）。 */
	size: number;
	/** 6 * size * size * 4 个 float。 */
	faces: Float32Array;
}

export interface PrefilteredCubemap {
	size: number;
	/** 从 size 到 1 的每一级，每级 6 个面连续存放；第 i 级的粗糙度是 i / (levels.length - 1)。 */
	levels: Float32Array[];
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

/** 工具模块不拆分，没有调试版和侧模块。 */
export async function createStrideTools(options: Omit<ModuleOptions, 'debug' | 'features'> = {}): Promise<StrideTools> {
	return new StrideTools(await instantiate(createStrideToolsCore, options));
}

export class StrideTools extends NativeModule<ToolsExport> {
	/**
	 * 导入 glTF/GLB（内嵌缓冲）或 FBX（转换成右手、Y 向上、米）。和 Stride 的模型导入一样，每个节点实例上的网格
	 * 按材质拆开，节点的世界变换烘焙进顶点。
	 */
	loadScene(bytes: Uint8Array, format: SceneFormat = 'auto'): Promise<Scene> {
		const code = enumCode(SCENE_FORMATS, format, 'format');
		return this.withArraysAsync([bytes], (data) =>
			this.awaitTask(this.module._st_scene_load(data, bytes.byteLength, code), 'st_scene_load',
				(task) => new Scene(this, task.result(0), task.result(1), task.result(2), task.result(3), task.result(4))));
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

	/** 解码 PNG、JPEG、TGA，得到 RGBA8 像素。 */
	decodeImage(bytes: Uint8Array): Promise<DecodedImage> {
		return this.withArraysAsync([bytes], (data) =>
			this.awaitTask(this.module._st_image_decode(data, bytes.byteLength), 'st_image_decode', (task) => ({
				width: task.result(0),
				height: task.result(1),
				channels: task.result(2),
				pixels: task.output(Uint8Array, 0),
			})));
	}

	/**
	 * 天空盒的镜面光照：GGX 预滤波成 outSize 的立方体贴图（同 Stride 的 RadiancePrefilteringGGX，默认 256、1024 个采样）。
	 * 最高一级是源贴图重采样，不滤波。
	 */
	prefilterCubemapGGX({ size, faces }: CubemapData, outSize = 256, samples = 1024): Promise<PrefilteredCubemap> {
		return this.withArraysAsync([faces], (f) =>
			this.awaitTask(this.module._st_cubemap_prefilter_ggx(f, size, outSize, samples), 'st_cubemap_prefilter_ggx', (task) => {
				const all = task.output(Float32Array, 0);
				const levels: Float32Array[] = [];
				for (let edge = outSize, offset = 0; edge >= 1; edge >>= 1) {
					levels.push(all.subarray(offset, offset + 6 * edge * edge * 4));
					offset += 6 * edge * edge * 4;
				}
				return { size: outSize, levels };
			}));
	}

	/** 天空盒的漫反射光照：3 阶球谐（同 Stride 的 LambertianPrefilteringSH），9 个 RGB 系数，已经乘过基函数常数。 */
	cubemapSH({ size, faces }: CubemapData): Promise<Float32Array> {
		return this.withArraysAsync([faces], (f) =>
			this.awaitTask(this.module._st_cubemap_sh_lambert(f, size), 'st_cubemap_sh_lambert', (task) => task.output(Float32Array, 0)));
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
	/** 材质名，MeshData.material 是这里的下标。 */
	readonly materials: readonly string[];
	readonly nodes: readonly SceneNode[];
	/** 动画名（FBX 的动画栈），bakeAnimation 的下标。 */
	readonly animations: readonly string[];

	/** @internal */
	constructor(tools: StrideTools, handle: number, meshCount: number, materialCount: number, nodeCount: number, animationCount: number) {
		super(tools, handle);
		const name = (ptr: number) => tools.module.UTF8ToString(ptr);
		this.meshCount = meshCount;
		this.materials = Array.from({ length: materialCount }, (_, i) => name(tools.module._st_scene_material_name(handle, i)));
		this.animations = Array.from({ length: animationCount }, (_, i) => name(tools.module._st_scene_animation_name(handle, i)));
		// st_node_info：parent、translation[3]、rotation[4]、scale[3]、world[16]，27 个字
		this.nodes = Array.from({ length: nodeCount }, (_, i) => tools.withScratch(27 * 4, (ptr) => {
			tools.check(tools.module._st_scene_node_info(handle, i, ptr), 'st_scene_node_info');
			const f = tools.f32(ptr, 27);
			return {
				name: name(tools.module._st_scene_node_name(handle, i)),
				parent: tools.i32(ptr, 1)[0]!,
				translation: [f[1]!, f[2]!, f[3]!],
				rotation: [f[4]!, f[5]!, f[6]!, f[7]!],
				scale: [f[8]!, f[9]!, f[10]!],
				world: f.slice(11, 27),
			} satisfies SceneNode;
		}));
	}

	mesh(index: number): MeshData {
		const t = this.native;
		return t.withScratch(28, (infoPtr) => {
			t.check(t.module._st_scene_mesh_info(this[HANDLE], index, infoPtr), 'st_scene_mesh_info');
			const info = t.u32(infoPtr, 7);
			const vertexCount = info[0]!, indexCount = info[1]!, hasNormals = !!info[2], hasUvs = !!info[3];
			const material = info[4] === NO_MATERIAL ? null : info[4]!;
			const node = info[5]!, jointCount = info[6]!;
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
					material,
					node,
					skin: jointCount > 0 ? this.#skin(index, vertexCount, jointCount) : null,
				};
			} finally {
				for (const ptr of [positions, normals, uvs, indices]) if (ptr) t.free(ptr);
			}
		});
	}

	#skin(index: number, vertexCount: number, jointCount: number): MeshSkin {
		const t = this.native;
		const buffers = [jointCount * 4, jointCount * 64, vertexCount * 8, vertexCount * 16].map((size) => t.alloc(size));
		const [jointNodes, inverseBinds, joints, weights] = buffers as [number, number, number, number];
		try {
			t.check(t.module._st_scene_mesh_skin(this[HANDLE], index, jointNodes, inverseBinds, joints, weights), 'st_scene_mesh_skin');
			return {
				jointNodes: t.copyOut(Uint32Array, jointNodes, jointCount),
				inverseBinds: t.copyOut(Float32Array, inverseBinds, jointCount * 16),
				joints: t.copyOut(Uint16Array, joints, vertexCount * 4),
				weights: t.copyOut(Float32Array, weights, vertexCount * 4),
			};
		} finally {
			for (const ptr of buffers) t.free(ptr);
		}
	}

	/** 把一个动画（只支持 FBX）按 sampleRate 重采样烘焙成每个节点的线性关键帧，时间从 0 起。 */
	bakeAnimation(index: number, sampleRate = 30): Promise<BakedAnimation> {
		const t = this.native;
		return t.awaitTask(t.module._st_scene_animation_bake(this[HANDLE], index, sampleRate), 'st_scene_animation_bake', (task) => {
			const counts = task.output(Uint32Array, 0);
			const keys = task.output(Float32Array, 1);
			const duration = task.output(Float32Array, 2)[0]!;
			let at = 0;
			const take = (count: number, width: number) => Array.from({ length: count }, () => {
				const key = Array.from(keys.subarray(at, at + width));
				at += width;
				return key;
			});
			const tracks = this.nodes.map((_, n) => ({
				translations: take(counts[n * 3]!, 4),
				rotations: take(counts[n * 3 + 1]!, 5),
				scales: take(counts[n * 3 + 2]!, 4),
			}));
			return { duration, tracks };
		});
	}

	protected override release(): void {
		this.native.module._st_scene_destroy(this[HANDLE]);
	}
}
