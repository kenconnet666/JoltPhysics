// stride-tools 资源管线模块的 JS API（Node）。耗时的操作返回原生任务背后的 Promise，可以同时处理多个资源：
// 一起发起、一起 await，它们共用模块的任务系统。
import createStrideToolsCore from './stride-tools-core.mjs';
import type { ToolsExport } from './stride-tools-core.mjs';
import { NAV_CONFIG_SIZE, NativeModule, NativeObject, HANDLE, enumCode, instantiate, writeNavConfig, type ModuleOptions, type NavConfig } from './native-module.ts';

export { NativeError, defaultWorkerThreads } from './native-module.ts';
export type { ModuleOptions, NativeErrorCode, NavConfig } from './native-module.ts';

/** auto 认得 glTF/GLB 和 FBX；OBJ 要显式给。 */
export type SceneFormat = 'auto' | 'gltf' | 'fbx' | 'obj';
export type BasisMode = 'uastc' | 'etc1s';

const SCENE_FORMATS: Record<SceneFormat, number> = { auto: 0, gltf: 1, fbx: 2, obj: 3 };
/** ST_NO_MATERIAL */
const NO_MATERIAL = 0xffffffff;
const BASIS_MODES: Record<BasisMode, number> = { uastc: 0, etc1s: 1 };

/** 材质的一个贴图槽：图片（Scene.image 的下标，没有时是 null）和 UV 变换（uv' = uv * scale + offset）。 */
export interface TextureSlot {
	image: number | null;
	scale: [number, number];
	offset: [number, number];
}

/** 材质的 PBR 参数（glTF 的金属度工作流；FBX 经 ufbx 换成 PBR）。颜色是线性的。 */
export interface MaterialInfo {
	baseColor: [number, number, number, number];
	metallic: number;
	roughness: number;
	/** 乘过自发光强度。 */
	emissive: [number, number, number];
	alphaMode: 'opaque' | 'mask' | 'blend';
	alphaCutoff: number;
	doubleSided: boolean;
	normalScale: number;
	baseColorTexture: TextureSlot;
	/** glTF 的打包：G 是粗糙度、B 是金属度。 */
	metallicRoughnessTexture: TextureSlot;
	normalTexture: TextureSlot;
	/** R 是遮蔽。 */
	occlusionTexture: TextureSlot;
	emissiveTexture: TextureSlot;
}

/** 材质引用的图片：嵌在文件里的有 data（编码过的字节，PNG、JPEG……），外部文件有 uri（相对源文件）；mime 不知道时是空串。 */
export interface SceneImage {
	data: Uint8Array | null;
	uri: string | null;
	mime: string;
}

/** st_material_info 的字数：base_color 4、metallic、roughness、emissive 3、alpha_mode、alpha_cutoff、double_sided、normal_scale，再 5 个贴图槽各 5 个。 */
const MATERIAL_INFO_WORDS = 13 + 5 * 5;

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
	/** 形变目标（glTF 的 targets、FBX 的 blend channel），没有时是空数组。 */
	morphTargets: MorphTarget[];
}

export interface MorphTarget {
	name: string;
	/** 每个顶点 xyz 的位置增量，和 positions 同一个空间。 */
	positions: Float32Array;
	/** 每个顶点 xyz 的法线增量；文件里没有时是 null。 */
	normals: Float32Array | null;
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
	/**
	 * 形变权重的曲线：按网格（有形变目标的，按 Scene.mesh 的下标从小到大）再按目标排，每个目标一组 [时间, 权重] 的线性关键帧；
	 * 动画没碰的目标是空数组。
	 */
	morphWeights: { mesh: number; target: number; keys: [number, number][] }[];
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

/** 关卡白盒的一个笔刷（st_brush_csg）：形状、运算、放在哪（列主序的 4x4 矩阵）。 */
export type Brush = {
	readonly operation: 'add' | 'subtract' | 'intersect';
	/** 列主序的 4x4 变换。 */
	readonly transform: ArrayLike<number>;
} & (
	| { readonly kind: 'box'; readonly size: readonly [number, number, number] }
	/** XZ 平面上的简单多边形（可以是凹的），从 y = 0 往上挤出 height。 */
	| { readonly kind: 'prism'; readonly points: readonly (readonly [number, number])[]; readonly height: number }
	/** 轴是 Y，中心在原点。 */
	| { readonly kind: 'cylinder'; readonly height: number; readonly bottomRadius: number; readonly topRadius: number; readonly segments: number }
	| { readonly kind: 'sphere'; readonly radius: number; readonly segments: number }
);

/** 笔刷合成的结果：合并过的顶点、三角形，每个三角形是哪个笔刷的面。 */
export interface BrushSolid {
	positions: Float32Array;
	indices: Uint32Array;
	/** 每个三角形一个，brushes 的下标。 */
	brushOfTriangle: Uint32Array;
}

/**
 * SDF 建模的一个形状（st_sdf_mesh）：按顺序和前面的合成（平滑的并、差、交），smoothness 是融合带的宽度（米，0 是硬接）。
 * transform 是列主序的 4x4（旋转、平移，可以带统一缩放）；形状在本地空间里以原点为中心。
 */
export type SdfShape = {
	readonly operation: 'add' | 'subtract' | 'intersect';
	readonly smoothness: number;
	readonly transform: ArrayLike<number>;
} & (
	| { readonly kind: 'sphere'; readonly radius: number }
	/** halfSize 是半边长，圆角在尺寸以内。 */
	| { readonly kind: 'box'; readonly halfSize: readonly [number, number, number]; readonly cornerRadius: number }
	/** 线段沿 Y，从 -halfHeight 到 halfHeight。 */
	| { readonly kind: 'capsule'; readonly halfHeight: number; readonly radius: number }
	| { readonly kind: 'cylinder'; readonly halfHeight: number; readonly radius: number; readonly edgeRadius: number }
	/** 环在 XZ 平面上。 */
	| { readonly kind: 'torus'; readonly ringRadius: number; readonly tubeRadius: number }
	| { readonly kind: 'ellipsoid'; readonly radii: readonly [number, number, number] }
	/** 圆台（顶半径 0 是圆锥）：轴是 Y，底面在 -halfHeight。 */
	| { readonly kind: 'cone'; readonly halfHeight: number; readonly bottomRadius: number; readonly topRadius: number }
	/** 两头是球的圆锥（四肢、角）：两个球心相距 height（沿 Y、中心在原点），半径差要小于 height。 */
	| { readonly kind: 'roundCone'; readonly height: number; readonly bottomRadius: number; readonly topRadius: number }
	/** 六棱柱：六边形在 XZ 平面上，radius 是中心到边的距离，轴是 Y。 */
	| { readonly kind: 'hexPrism'; readonly halfHeight: number; readonly radius: number }
);

/** SDF 建模的结果：光滑的法线（距离场的梯度），每个三角形离哪个形状的表面最近。 */
export interface SdfSolid {
	positions: Float32Array;
	normals: Float32Array;
	indices: Uint32Array;
	/** 每个三角形一个，shapes 的下标。 */
	shapeOfTriangle: Uint32Array;
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
	/** 编码器自己开线程，默认开：量过（1K 贴图 19.6 → 3.9 秒；2K 单线程 76 秒），24 张 512² 并发时总耗时也更短（9.4 → 6.6 秒）。 */
	threaded?: boolean;
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

/** 字体的度量（字体单位）和名字，见 Tools.h 的 st_font_description。 */
export interface FontDescription {
	/** 字体集合（.ttc）里的字体数，普通的 .ttf/.otf 是 1。 */
	readonly fontCount: number;
	readonly glyphCount: number;
	readonly unitsPerEm: number;
	/** head 表的 xMin、yMin、xMax、yMax。 */
	readonly bounds: readonly [number, number, number, number];
	/** hhea 的（descent 是负数）。 */
	readonly ascent: number;
	readonly descent: number;
	readonly lineGap: number;
	/** OS/2 的 fsSelection 第 7 位：排版时该用 typo 那一组。 */
	readonly useTypoMetrics: boolean;
	readonly typoAscent: number;
	readonly typoDescent: number;
	readonly typoLineGap: number;
	/** OS/2 的（winDescent 是正数）。 */
	readonly winAscent: number;
	readonly winDescent: number;
	/** OS/2 版本 2 起才有，没有时是 0。 */
	readonly xHeight: number;
	readonly capHeight: number;
	/** head 表的 flags：第 3 位要求栅格化时把 ppem 取整（FreeType 开 hinting 时照做）。 */
	readonly headFlags: number;
	/** 名字表的 1、2 号（"Arial"、"Bold Italic"）。 */
	readonly family: string;
	readonly subfamily: string;
	/** 名字表的 16、17 号，没有时是空串。 */
	readonly typographicFamily: string;
	readonly typographicSubfamily: string;
}

/** 字体映射的码位和 kern 表。 */
export interface FontCodepoints {
	/** 升序。 */
	readonly codepoints: Uint32Array;
	/** 每个码位的字形下标。 */
	readonly glyphs: Uint32Array;
	/** kern 表的字距，每三个一组：glyph1、glyph2、调整量（字体单位），按 glyph1、glyph2 排序。 */
	readonly kerning: Int32Array;
}

/** 一个字形的轮廓：二次贝塞尔曲线（x1 y1 x2 y2 x3 y3，字体单位，y 向上），直线是控制点等于终点的二次曲线。 */
export interface GlyphOutline {
	readonly glyph: number;
	readonly advance: number;
	readonly leftBearing: number;
	/** 轮廓的 x0、y0、x1、y1；没有轮廓（空格）时都是 0。 */
	readonly bounds: readonly [number, number, number, number];
	readonly curves: Float32Array;
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
	 * 导入 glTF/GLB（内嵌缓冲）、FBX（转换成右手、Y 向上、米）或 OBJ（不读 .mtl、不转换）。和 Stride 的模型导入一样，每个节点实例上的网格
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

	/** 关卡白盒：按顺序把笔刷加上、减掉、求交（Manifold），得到一个封闭的网格。 */
	brushCsg(brushes: readonly Brush[]): Promise<BrushSolid> {
		const KINDS = { box: 0, prism: 1, cylinder: 2, sphere: 3 } as const;
		const OPERATIONS = { add: 0, subtract: 1, intersect: 2 } as const;
		const stream: number[] = [];
		for (const brush of brushes) {
			if (brush.transform.length !== 16) throw new Error('brushCsg: a brush transform needs 16 numbers');
			stream.push(KINDS[brush.kind], OPERATIONS[brush.operation], ...Array.from(brush.transform));
			switch (brush.kind) {
				case 'box': stream.push(...brush.size); break;
				case 'prism': stream.push(brush.height, brush.points.length, ...brush.points.flat()); break;
				case 'cylinder': stream.push(brush.height, brush.bottomRadius, brush.topRadius, brush.segments); break;
				case 'sphere': stream.push(brush.radius, brush.segments); break;
			}
		}
		const floats = Float32Array.from(stream);
		return this.withArraysAsync([floats], (f) =>
			this.awaitTask(this.module._st_brush_csg(f, floats.length), 'st_brush_csg', (task) => ({
				positions: task.output(Float32Array, 0),
				indices: task.output(Uint32Array, 1),
				brushOfTriangle: task.output(Uint32Array, 2),
			})));
	}

	/** SDF 建模：形状按顺序平滑合成，按大约 edgeLength 的边长网格化（Manifold 的等值面）。 */
	sdfMesh(shapes: readonly SdfShape[], edgeLength: number): Promise<SdfSolid> {
		const KINDS = { sphere: 0, box: 1, capsule: 2, cylinder: 3, torus: 4, ellipsoid: 5, cone: 6, roundCone: 7, hexPrism: 8 } as const;
		const OPERATIONS = { add: 0, subtract: 1, intersect: 2 } as const;
		const stream: number[] = [];
		for (const shape of shapes) {
			if (shape.transform.length !== 16) throw new Error('sdfMesh: a shape transform needs 16 numbers');
			stream.push(KINDS[shape.kind], OPERATIONS[shape.operation], shape.smoothness, ...Array.from(shape.transform));
			switch (shape.kind) {
				case 'sphere': stream.push(shape.radius); break;
				case 'box': stream.push(...shape.halfSize, shape.cornerRadius); break;
				case 'capsule': stream.push(shape.halfHeight, shape.radius); break;
				case 'cylinder': stream.push(shape.halfHeight, shape.radius, shape.edgeRadius); break;
				case 'torus': stream.push(shape.ringRadius, shape.tubeRadius); break;
				case 'ellipsoid': stream.push(...shape.radii); break;
				case 'cone': stream.push(shape.halfHeight, shape.bottomRadius, shape.topRadius); break;
				case 'roundCone': stream.push(shape.height, shape.bottomRadius, shape.topRadius); break;
				case 'hexPrism': stream.push(shape.halfHeight, shape.radius); break;
			}
		}
		const floats = Float32Array.from(stream);
		return this.withArraysAsync([floats], (f) =>
			this.awaitTask(this.module._st_sdf_mesh(f, floats.length, edgeLength), 'st_sdf_mesh', (task) => ({
				positions: task.output(Float32Array, 0),
				normals: task.output(Float32Array, 1),
				indices: task.output(Uint32Array, 2),
				shapeOfTriangle: task.output(Uint32Array, 3),
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

	/** 用 Lanczos3 把 RGBA8 的图片缩放到 outWidth x outHeight（同 Stride 的 TextureTool.Resize），边缘钳住。 */
	resizeImage(rgba: Uint8Array, width: number, height: number, outWidth: number, outHeight: number): Promise<Uint8Array> {
		return this.withArraysAsync([rgba], (pixels) =>
			this.awaitTask(this.module._st_image_resize(pixels, width, height, outWidth, outHeight), 'st_image_resize',
				(task) => task.output(Uint8Array, 0)));
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
	encodeBasis(rgba: Uint8Array, width: number, height: number, { mode = 'uastc', srgb = true, mipmaps = true, quality = 75, effort = 2, threaded = true }: BasisOptions = {}): Promise<Uint8Array> {
		const code = enumCode(BASIS_MODES, mode, 'BasisOptions.mode');
		const flags = (srgb ? 1 : 0) | (mipmaps ? 2 : 0) | (threaded ? 4 : 0);
		return this.withArraysAsync([rgba], (pixels) =>
			this.awaitTask(this.module._st_basis_encode(pixels, width, height, code, flags, quality, effort), 'st_basis_encode',
				(task) => task.output(Uint8Array, 0)));
	}

	/** 字体文件（或集合里第 index 个字体）的度量和名字。 */
	describeFont(font: Uint8Array, index = 0): FontDescription {
		const SIZE = 19 * 4 + 4 * 128;
		return this.withArrays([font], (f) => this.withScratch(SIZE, (out) => {
			this.check(this.module._st_font_describe(f, font.byteLength, index, out), 'st_font_describe');
			const i = this.i32(out, 19);
			const text = (at: number) => this.module.UTF8ToString(out + 19 * 4 + at * 128);
			return {
				fontCount: i[0]!, glyphCount: i[1]!, unitsPerEm: i[2]!, bounds: [i[3]!, i[4]!, i[5]!, i[6]!],
				ascent: i[7]!, descent: i[8]!, lineGap: i[9]!, useTypoMetrics: i[10] !== 0,
				typoAscent: i[11]!, typoDescent: i[12]!, typoLineGap: i[13]!, winAscent: i[14]!, winDescent: i[15]!,
				xHeight: i[16]!, capHeight: i[17]!, headFlags: i[18]!,
				family: text(0), subfamily: text(1), typographicFamily: text(2), typographicSubfamily: text(3),
			};
		}));
	}

	/** 字体映射的所有码位、对应的字形和 kern 表。 */
	fontCodepoints(font: Uint8Array, index = 0): Promise<FontCodepoints> {
		return this.withArraysAsync([font], (f) =>
			this.awaitTask(this.module._st_font_codepoints(f, font.byteLength, index), 'st_font_codepoints', (task) => ({
				codepoints: task.output(Uint32Array, 0),
				glyphs: task.output(Uint32Array, 1),
				kerning: task.output(Int32Array, 2),
			})));
	}

	/** 字形的轮廓（并行）；三次曲线（CFF）按 tolerance（字体单位）拆成二次。 */
	glyphOutlines(font: Uint8Array, glyphs: Iterable<number>, { index = 0, tolerance = 0.5 }: { index?: number; tolerance?: number } = {}): Promise<GlyphOutline[]> {
		const list = new Uint32Array([...glyphs]);
		return this.withArraysAsync([font, list], (f, g) =>
			this.awaitTask(this.module._st_font_outlines(f, font.byteLength, index, g, list.length, tolerance), 'st_font_outlines', (task) => {
				// st_glyph_outline：glyph、advance、left_bearing、bounds[4]、curve_offset、curve_count
				const records = task.output(Int32Array, 0);
				const curves = task.output(Float32Array, 1);
				return Array.from(list, (glyph, n) => {
					const r = records.subarray(n * 9, n * 9 + 9);
					return {
						glyph,
						advance: r[1]!,
						leftBearing: r[2]!,
						bounds: [r[3]!, r[4]!, r[5]!, r[6]!],
						curves: curves.slice(r[7]! * 6, (r[7]! + r[8]!) * 6),
					} satisfies GlyphOutline;
				});
			}));
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
	/** 材质的参数，和 materials 一一对应。 */
	readonly materialInfos: readonly MaterialInfo[];
	/** 材质的贴图引用的图片数（Scene.image 的下标从 0 到它）。 */
	readonly imageCount: number;
	readonly nodes: readonly SceneNode[];
	/** 动画名（glTF 的动画、FBX 的动画栈），bakeAnimation 的下标。 */
	readonly animations: readonly string[];

	/** @internal */
	constructor(tools: StrideTools, handle: number, meshCount: number, materialCount: number, nodeCount: number, animationCount: number) {
		super(tools, handle);
		const name = (ptr: number) => tools.module.UTF8ToString(ptr);
		this.meshCount = meshCount;
		this.materials = Array.from({ length: materialCount }, (_, i) => name(tools.module._st_scene_material_name(handle, i)));
		this.animations = Array.from({ length: animationCount }, (_, i) => name(tools.module._st_scene_animation_name(handle, i)));
		this.materialInfos = Array.from({ length: materialCount }, (_, i) => tools.withScratch(MATERIAL_INFO_WORDS * 4, (ptr) => {
			tools.check(tools.module._st_scene_material_info(handle, i, ptr), 'st_scene_material_info');
			const f = tools.f32(ptr, MATERIAL_INFO_WORDS), w = tools.i32(ptr, MATERIAL_INFO_WORDS);
			const slot = (at: number): TextureSlot => ({ image: w[at]! < 0 ? null : w[at]!, scale: [f[at + 1]!, f[at + 2]!], offset: [f[at + 3]!, f[at + 4]!] });
			return {
				baseColor: [f[0]!, f[1]!, f[2]!, f[3]!],
				metallic: f[4]!,
				roughness: f[5]!,
				emissive: [f[6]!, f[7]!, f[8]!],
				alphaMode: (['opaque', 'mask', 'blend'] as const)[w[9]!] ?? 'opaque',
				alphaCutoff: f[10]!,
				doubleSided: w[11] !== 0,
				normalScale: f[12]!,
				baseColorTexture: slot(13),
				metallicRoughnessTexture: slot(18),
				normalTexture: slot(23),
				occlusionTexture: slot(28),
				emissiveTexture: slot(33),
			} satisfies MaterialInfo;
		}));
		this.imageCount = tools.check(tools.module._st_scene_image_count(handle), 'st_scene_image_count');
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

	/** 第 index 张图片：嵌在文件里的拷出字节，外部文件给路径。 */
	image(index: number): SceneImage {
		const t = this.native;
		return t.withScratch(24, (out) => {
			const size = t.check(t.module._st_scene_image(this[HANDLE], index, out, out + 8, out + 16), 'st_scene_image');
			const [data, uri, mime] = [0, 8, 16].map((offset) => t.readPointer(out + offset)) as [number, number, number];
			const path = t.module.UTF8ToString(uri);
			return { data: size > 0 ? t.copyOut(Uint8Array, data, size) : null, uri: size > 0 || path === '' ? null : path, mime: t.module.UTF8ToString(mime) };
		});
	}

	mesh(index: number): MeshData {
		const t = this.native;
		return t.withScratch(36, (infoPtr) => {
			t.check(t.module._st_scene_mesh_info(this[HANDLE], index, infoPtr), 'st_scene_mesh_info');
			const info = t.u32(infoPtr, 9);
			const vertexCount = info[0]!, indexCount = info[1]!, hasNormals = !!info[2], hasUvs = !!info[3];
			const material = info[4] === NO_MATERIAL ? null : info[4]!;
			const node = info[5]!, jointCount = info[6]!, morphTargetCount = info[7]!, morphHasNormals = !!info[8];
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
					morphTargets: Array.from({ length: morphTargetCount }, (_, target) => this.#morphTarget(index, target, vertexCount, morphHasNormals)),
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

	#morphTarget(index: number, target: number, vertexCount: number, hasNormals: boolean): MorphTarget {
		const t = this.native;
		const positions = t.alloc(vertexCount * 12);
		const normals = hasNormals ? t.alloc(vertexCount * 12) : 0;
		try {
			t.check(t.module._st_scene_mesh_morph(this[HANDLE], index, target, positions, normals), 'st_scene_mesh_morph');
			return {
				name: t.module.UTF8ToString(t.module._st_scene_mesh_morph_name(this[HANDLE], index, target)),
				positions: t.copyOut(Float32Array, positions, vertexCount * 3),
				normals: hasNormals ? t.copyOut(Float32Array, normals, vertexCount * 3) : null,
			};
		} finally {
			t.free(positions);
			if (normals) t.free(normals);
		}
	}

	/** 把一个动画按 sampleRate 重采样烘焙成每个节点的线性关键帧和形变权重的曲线，时间从 0 起。 */
	bakeAnimation(index: number, sampleRate = 30): Promise<BakedAnimation> {
		const t = this.native;
		// 有形变目标的网格和它们的目标数，对上烘焙结果里权重曲线的顺序
		const morphMeshes = Array.from({ length: this.meshCount }, (_, mesh) => t.withScratch(36, (infoPtr) => {
			t.check(t.module._st_scene_mesh_info(this[HANDLE], mesh, infoPtr), 'st_scene_mesh_info');
			return { mesh, targets: t.u32(infoPtr, 9)[7]! };
		})).filter((m) => m.targets > 0);
		return t.awaitTask(t.module._st_scene_animation_bake(this[HANDLE], index, sampleRate), 'st_scene_animation_bake', (task) => {
			const counts = task.output(Uint32Array, 0);
			const keys = task.output(Float32Array, 1);
			const duration = task.output(Float32Array, 2)[0]!;
			const weightCounts = task.output(Uint32Array, 3);
			const weightKeys = task.output(Float32Array, 4);
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
			let weightAt = 0;
			let curve = 0;
			const morphWeights = morphMeshes.flatMap(({ mesh, targets }) => Array.from({ length: targets }, (_, target) => {
				const keys = Array.from({ length: weightCounts[curve++]! }, () => {
					const key: [number, number] = [weightKeys[weightAt]!, weightKeys[weightAt + 1]!];
					weightAt += 2;
					return key;
				});
				return { mesh, target, keys };
			}));
			return { duration, tracks, morphWeights };
		});
	}

	protected override release(): void {
		this.native.module._st_scene_destroy(this[HANDLE]);
	}
}
