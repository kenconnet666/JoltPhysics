// JS API of the stride-tools pipeline module (Node). See stride-tools.d.ts.
// Heavy operations return promises backed by native tasks, so assets can be processed concurrently:
// start several and await them together; they share the module's job system.
import createStrideToolsCore from './stride-tools-core.mjs';
import { NativeModule, NAV_CONFIG_SIZE, instantiate, writeNavConfig } from './native-module.mjs';

export { NativeError, defaultWorkerThreads } from './native-module.mjs';

export const SceneFormat = Object.freeze({ auto: 0, gltf: 1, fbx: 2 });
export const BasisMode = Object.freeze({ uastc: 0, etc1s: 1 });

export async function createStrideTools(options = {}) {
	return new StrideTools(await instantiate(createStrideToolsCore, options));
}

export class StrideTools extends NativeModule {
	/** Imports a glTF/GLB (embedded buffers) or FBX file. */
	loadScene(bytes, format = SceneFormat.auto) {
		return this.withArraysAsync([bytes], (data) =>
			this.awaitTask(this.module._st_scene_load(data, bytes.byteLength, format), 'st_scene_load',
				(task) => new Scene(this, task.result(0), task.result(1))));
	}

	/** MikkTSpace tangents: one xyzw per index (corner). */
	generateTangents({ positions, normals, uvs, indices }) {
		return this.withArrays([positions, normals, uvs, indices], (p, n, u, i) => this.withScratch(indices.length * 16, (out) => {
			this.check(this.module._st_generate_tangents(p, n, u, i, indices.length, out), 'st_generate_tangents');
			return this.copyOut(Float32Array, out, indices.length * 4);
		}));
	}

	/** Vertex cache plus overdraw optimization; returns the reordered indices. */
	optimizeMesh(indices, positions) {
		return this.withArrays([indices, positions], (i, p) => {
			this.check(this.module._st_meshopt_optimize(i, indices.length, p, positions.length / 3, 12), 'st_meshopt_optimize');
			return this.copyOut(Uint32Array, i, indices.length);
		});
	}

	/** Returns { indices, error }. */
	simplifyMesh(indices, positions, targetIndexCount, targetError = 0.01) {
		return this.withArrays([indices, positions], (i, p) => this.withScratch(indices.length * 4 + 4, (out) => {
			const errorPtr = out + indices.length * 4;
			const count = this.check(this.module._st_meshopt_simplify(out, i, indices.length, p, positions.length / 3, 12, targetIndexCount, targetError, errorPtr), 'st_meshopt_simplify');
			return { indices: this.copyOut(Uint32Array, out, count), error: this.f32(errorPtr, 1)[0] };
		}));
	}

	/** meshopt vertex codec (glTF EXT_meshopt_compression "ATTRIBUTES"). */
	encodeVertices(vertices, vertexSize) {
		const count = vertices.byteLength / vertexSize;
		const capacity = this.module._st_meshopt_encode_vertex_bound(count, vertexSize);
		return this.withArrays([vertices], (v) => this.withScratch(capacity, (out) => {
			const size = this.module._st_meshopt_encode_vertices(out, capacity, v, count, vertexSize);
			return this.copyOut(Uint8Array, out, this.check(size > 0 ? size : -3, 'st_meshopt_encode_vertices'));
		}));
	}

	/** meshopt index codec (glTF EXT_meshopt_compression "TRIANGLES"). */
	encodeIndices(indices, vertexCount) {
		const capacity = this.module._st_meshopt_encode_index_bound(indices.length, vertexCount);
		return this.withArrays([indices], (i) => this.withScratch(capacity, (out) => {
			const size = this.module._st_meshopt_encode_indices(out, capacity, i, indices.length);
			return this.copyOut(Uint8Array, out, this.check(size > 0 ? size : -3, 'st_meshopt_encode_indices'));
		}));
	}

	/** Lightmap UVs. Returns { xref, uvs, indices, width, height }; xref maps new vertices to input vertices. */
	unwrapUvs({ positions, normals, indices }, texelsPerUnit = 0) {
		return this.withArraysAsync([positions, normals, indices], (p, n, i) =>
			this.awaitTask(this.module._st_uv_unwrap(p, n, positions.length / 3, i, indices.length, texelsPerUnit), 'st_uv_unwrap', (task) => ({
				xref: task.output(Uint32Array, 0),
				uvs: task.output(Float32Array, 1),
				indices: task.output(Uint32Array, 2),
				width: task.result(0),
				height: task.result(1),
			})));
	}

	/** Convex decomposition for physics. Returns [{ vertices: Float32Array, triangles: Uint32Array }]. */
	convexDecompose({ positions, indices }, { maxHulls = 16, voxelResolution = 100000, maxVerticesPerHull = 64 } = {}) {
		return this.withArraysAsync([positions, indices], (p, i) =>
			this.awaitTask(this.module._st_convex_decompose(p, positions.length / 3, i, indices.length, maxHulls, voxelResolution, maxVerticesPerHull), 'st_convex_decompose', (task) => {
				const counts = task.output(Int32Array, 0);
				const points = task.output(Float32Array, 1);
				const triangles = task.output(Uint32Array, 2);
				const hulls = [];
				for (let h = 0, point = 0, triangle = 0; h < task.result(0); ++h) {
					const vertexCount = counts[h * 2], triangleCount = counts[h * 2 + 1];
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

	/** Encodes RGBA8 pixels into a KTX2 file. */
	encodeBasis(rgba, width, height, { mode = BasisMode.uastc, srgb = true, mipmaps = true, quality = 75, effort = 2 } = {}) {
		const flags = (srgb ? 1 : 0) | (mipmaps ? 2 : 0);
		return this.withArraysAsync([rgba], (pixels) =>
			this.awaitTask(this.module._st_basis_encode(pixels, width, height, mode, flags, quality, effort), 'st_basis_encode',
				(task) => task.output(Uint8Array, 0)));
	}

	/**
	 * MSDF glyphs, generated in parallel. Returns [{ codepoint, width, height, bearingX, bearingY,
	 * advance, pixels }] with RGB8 pixels, rows top-down; pixels is empty for glyphs without outline.
	 */
	msdfGlyphs(font, codepoints, pixelSize = 32, distanceRange = 4) {
		const points = new Uint32Array(codepoints);
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

	/** Builds an ozz skeleton archive. joints: [{ parent, translation, rotation (xyzw), scale, name }]. */
	buildSkeleton(joints) {
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

	/**
	 * Builds an ozz animation archive. tracks: one per joint, { translations: [[t, x, y, z]],
	 * rotations: [[t, x, y, z, w]], scales: [[t, x, y, z]] }, times in seconds.
	 */
	buildAnimation(duration, tracks) {
		const counts = new Uint32Array(tracks.length * 3);
		const keys = [];
		tracks.forEach((track, j) => {
			['translations', 'rotations', 'scales'].forEach((kind, k) => {
				const list = track[kind] ?? [];
				counts[j * 3 + k] = list.length;
				list.forEach((key) => keys.push(...key));
			});
		});
		return this.withArrays([counts, new Float32Array(keys)], (c, k) =>
			this.readOwnedBuffer((data, size) => this.module._st_anim_build_animation(duration, tracks.length, c, k, data, size), 'st_anim_build_animation'));
	}

	/** Bakes a navmesh for StrideNative.loadNavMesh; tiles are built in parallel. */
	bakeNavMesh(vertices, triangles, config = {}) {
		return this.withArraysAsync([vertices, triangles], (v, t) => this.withScratch(NAV_CONFIG_SIZE, (cfg) => {
			writeNavConfig(this, cfg, config);
			return this.awaitTask(this.module._st_nav_bake(v, vertices.length / 3, t, triangles.length / 3, cfg), 'st_nav_bake',
				(task) => task.output(Uint8Array, 0));
		}));
	}
}

export class Scene {
	constructor(tools, handle, meshCount) {
		this.tools = tools;
		this.handle = handle;
		this.meshCount = meshCount;
	}

	/** Returns { positions, normals?, uvs?, indices } for one mesh. */
	mesh(index) {
		const t = this.tools;
		return t.withScratch(16, (infoPtr) => {
			t.check(t.module._st_scene_mesh_info(this.handle, index, infoPtr), 'st_scene_mesh_info');
			const [vertexCount, indexCount, hasNormals, hasUvs] = t.u32(infoPtr, 4);
			const positions = t.alloc(vertexCount * 12);
			const normals = hasNormals ? t.alloc(vertexCount * 12) : 0;
			const uvs = hasUvs ? t.alloc(vertexCount * 8) : 0;
			const indices = t.alloc(indexCount * 4);
			try {
				t.check(t.module._st_scene_mesh_copy(this.handle, index, positions, normals, uvs, indices), 'st_scene_mesh_copy');
				return {
					positions: t.copyOut(Float32Array, positions, vertexCount * 3),
					normals: hasNormals ? t.copyOut(Float32Array, normals, vertexCount * 3) : null,
					uvs: hasUvs ? t.copyOut(Float32Array, uvs, vertexCount * 2) : null,
					indices: t.copyOut(Uint32Array, indices, indexCount),
				};
			} finally {
				[positions, normals, uvs, indices].forEach((ptr) => ptr && t.free(ptr));
			}
		});
	}

	destroy() {
		this.tools.module._st_scene_destroy(this.handle);
	}
}
