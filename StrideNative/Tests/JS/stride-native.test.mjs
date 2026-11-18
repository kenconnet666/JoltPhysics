// End-to-end tests of both wasm modules in Node: tools produce assets, the runtime consumes them.
// Run through Scripts/build-wasm.ps1, which sets STRIDE_NATIVE_DIST to the build's dist folder.
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { after, before, describe, it } from 'node:test';
import { fileURLToPath, pathToFileURL } from 'node:url';
import path from 'node:path';

const dist = process.env.STRIDE_NATIVE_DIST;
assert.ok(dist, 'STRIDE_NATIVE_DIST must point at the build dist folder');
// 绑定是 TypeScript：Node 24 直接去掉类型运行
const { createStrideNative, NativeError } = await import(pathToFileURL(path.join(dist, 'stride-native.ts')));
const { createStrideTools } = await import(pathToFileURL(path.join(dist, 'stride-tools.ts')));

const testData = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'Data');
const fontPath = process.env.STRIDE_TEST_FONT ?? 'C:/Windows/Fonts/arial.ttf';

/** Flat square grid on y = 0, centered on the origin. */
function ground(size, cells) {
	const vertices = [], triangles = [];
	for (let z = 0; z <= cells; ++z)
		for (let x = 0; x <= cells; ++x)
			vertices.push(-size / 2 + (size * x) / cells, 0, -size / 2 + (size * z) / cells);
	for (let z = 0; z < cells; ++z)
		for (let x = 0; x < cells; ++x) {
			const i = z * (cells + 1) + x;
			triangles.push(i, i + cells + 1, i + 1, i + 1, i + cells + 1, i + cells + 2);
		}
	return { vertices: new Float32Array(vertices), triangles: new Int32Array(triangles) };
}

describe('stride-native and stride-tools', () => {
	let native, tools;

	before(async () => {
		[native, tools] = await Promise.all([createStrideNative({ workerThreads: 4 }), createStrideTools({ workerThreads: 4 })]);
	});

	after(() => {
		native?.shutdown();
		tools?.shutdown();
	});

	it('starts the requested worker threads', () => {
		assert.equal(native.workerThreadCount, 4);
		assert.ok(native.buffer instanceof SharedArrayBuffer);
	});

	it('views native memory as int64 for decimal and date-time columns', () => {
		const ptr = native.alloc(4 * 8);
		try {
			const values = native.i64(ptr, 4);
			values.set([-(2n ** 63n), 2n ** 63n - 1n, 1_706_679_900_000_000_000n, -1n]);
			// 同一块内存按字节看，最后一个 -1 是 8 个 0xff：确认是小端的 int64，和 C 的 int64_t 一致
			assert.deepEqual([...native.u8(ptr + 24, 8)], [255, 255, 255, 255, 255, 255, 255, 255]);
			assert.equal(native.u64(ptr, 4)[3], 2n ** 64n - 1n);
			assert.equal(native.i64(ptr, 4)[2], 1_706_679_900_000_000_000n);
		} finally {
			native.free(ptr);
		}
	});

	it('steps physics on the shared job system', () => {
		const world = native.createPhysicsWorld(64);
		assert.equal(world.addBox({ position: [0, -1, 0], halfExtent: [50, 1, 50], motion: 'static' }), 0);
		const boxes = Array.from({ length: 20 }, (_, i) => ({ position: [(i % 5) * 2, 2 + Math.floor(i / 5) * 2, 0], halfExtent: [0.5, 0.5, 0.5] }));
		assert.deepEqual(world.addBoxes(boxes), Array.from({ length: 20 }, (_, i) => i + 1));
		const positions = world.positions();
		assert.equal(positions.length, 21 * 3);
		assert.equal(positions[1 * 3 + 1], 2, 'positions are written when bodies are added');
		for (let i = 0; i < 240; ++i)
			world.step(1 / 60);
		assert.equal(world.positions(), positions, 'the view is reused, step() rewrites it in place');
		for (let body = 1; body <= 20; ++body)
			assert.ok(positions[body * 3 + 1] > 0.3 && positions[body * 3 + 1] < 8, `body ${body} rests on the floor or stack`);
		assert.throws(() => world.addBox({ position: [0, 0, 0], halfExtent: [1, 1, 1], motion: 'kinematic' }), /motion/);
		world.destroy();
		world.destroy(); // 重复释放没有效果
		assert.ok(world.destroyed);
		assert.throws(() => world.step(1 / 60), (error) => error instanceof NativeError && error.code === 'invalid-handle' && /PhysicsWorld was destroyed/.test(error.message));
	});

	it('propagates transforms parent-first and writes physics poses back into slots', () => {
		using transforms = native.createTransforms(8);
		const [root, parent, child] = transforms.allocate(3);
		assert.deepEqual([root, parent, child], [1, 2, 3]);
		assert.equal(transforms.slotLimit, 3);
		const { position, rotation, world } = transforms;
		// 父节点平移 (10, 0, 0)、绕 Y 转 90°；子节点本地 (0, 4, 2) 转成 (2, 4, 0)，世界位置 (12, 4, 0)
		const s = Math.SQRT1_2;
		position[root * 3 + 1] = 5;
		position[parent * 3] = 10;
		rotation.set([0, s, 0, s], parent * 4);
		transforms.setParent(child, parent);
		position.set([0, 4, 2], child * 3);
		transforms.update();
		const near = (actual, expected, epsilon = 1e-4) => assert.ok(Math.abs(actual - expected) < epsilon, `${actual} is not near ${expected}`);
		[12, 4, 0].forEach((v, i) => near(world[child * 16 + 12 + i], v));

		using physics = native.createPhysicsWorld(8, { transforms });
		physics.addBox({ position: [0, -1, 0], halfExtent: [50, 1, 50], motion: 'static' });
		physics.addBoxes([root, child].map((transform) => ({ halfExtent: [0.5, 0.5, 0.5], transform })));
		for (let i = 0; i < 180; ++i) {
			physics.step(1 / 60);
			transforms.update();
		}
		near(position[root * 3 + 1], 0.5, 0.05);
		// 写回的是父节点空间里的位姿：世界里落在 (12, 0.5, 0)
		[12, 0.5, 0].forEach((v, i) => near(world[child * 16 + 12 + i], v, 0.05));

		assert.throws(() => transforms.setParent(parent, child), (error) => error instanceof NativeError && error.code === 'invalid-argument' && /descendant of itself/.test(error.message));
		assert.throws(() => transforms.free(parent), /still has 1 child/);
		assert.throws(() => native.createPhysicsWorld(2).addBox({ halfExtent: [1, 1, 1], transform: root }), /without a transform store/);
		assert.throws(() => transforms.allocate(6), /capacity 8 reached/);
		assert.equal(transforms.allocate(5).length, 5);
	});

	it('releases native objects with using and reports failures as NativeError codes', () => {
		let escaped;
		{
			using world = native.createPhysicsWorld(4);
			escaped = world;
			assert.equal(world.destroyed, false);
		}
		assert.equal(escaped.destroyed, true);
		assert.throws(() => native.loadNavMesh(new Uint8Array(16)), (error) => error instanceof NativeError && error.code === 'failed' && error.status === -3);
		assert.throws(() => native.transcodeKtx2(new Uint8Array(4), 'png'), /target must be one of 'rgba8', 'bc7', 'astc4x4', 'etc2rgba8', not 'png'/);
		assert.throws(() => native.decodeMeshopt({ source: new Uint8Array(4), count: 1, stride: 4, filter: 'zip' }), /MeshoptRequest.filter/);
	});

	it('round-trips meshopt encoding (tools) and parallel decoding (runtime)', async () => {
		const vertexCount = 5000, stride = 16;
		const vertices = new Float32Array(vertexCount * 4).map((_, i) => Math.sin(i * 0.37));
		const indices = new Uint32Array(Array.from({ length: (vertexCount - 2) * 3 }, (_, i) => Math.floor(i / 3) + (i % 3)));
		const encodedVertices = tools.encodeVertices(new Uint8Array(vertices.buffer), stride);
		const encodedIndices = tools.encodeIndices(indices, vertexCount);
		assert.ok(encodedVertices.length < vertices.byteLength);

		const [decodedVertices, decodedIndices] = await native.decodeMeshoptBatch([
			{ source: encodedVertices, count: vertexCount, stride, mode: 'attributes' },
			{ source: encodedIndices, count: indices.length, stride: 4, mode: 'triangles' },
		]);
		assert.deepEqual(new Float32Array(decodedVertices.buffer), vertices);
		const triangles = (array) => {
			const out = [];
			for (let i = 0; i < array.length; i += 3) {
				const t = [array[i], array[i + 1], array[i + 2]];
				while (t[0] > t[1] || t[0] > t[2]) t.push(t.shift()); // the codec may rotate corners
				out.push(t.join());
			}
			return out;
		};
		assert.deepEqual(triangles(new Uint32Array(decodedIndices.buffer)), triangles(indices));
	});

	it('encodes KTX2 (tools) and transcodes every mip level in parallel (runtime)', async () => {
		const size = 128;
		const rgba = new Uint8Array(size * size * 4);
		for (let i = 0; i < size * size; ++i)
			rgba.set([(i % size) * 2, Math.floor(i / size) * 2, 100, 255], i * 4);
		const ktx2 = await tools.encodeBasis(rgba, size, size, { mode: 'uastc', mipmaps: true });
		const info = native.ktx2Info(ktx2);
		assert.equal(info.width, size);
		assert.equal(info.levels, 8);

		const rgbaResult = await native.transcodeKtx2(ktx2, 'rgba8');
		assert.equal(rgbaResult.levels.length, 8);
		assert.equal(rgbaResult.levels[0].length, size * size * 4);
		// UASTC is lossy but close: compare a pixel in the middle.
		const p = (64 * size + 64) * 4;
		assert.ok(Math.abs(rgbaResult.levels[0][p] - 128) < 12 && Math.abs(rgbaResult.levels[0][p + 1] - 128) < 12);

		const bc7 = await native.transcodeKtx2(ktx2, 'bc7');
		assert.equal(bc7.unitSize, 16);
		assert.equal(bc7.levels[0].length, (size / 4) * (size / 4) * 16);
	});

	it('bakes a navmesh (tools), loads it, finds paths and drives a crowd (runtime)', async () => {
		const { vertices, triangles } = ground(60, 30);
		const baked = await tools.bakeNavMesh(vertices, triangles, { tileSize: 32 });
		const navMesh = native.loadNavMesh(baked);
		const path = navMesh.findPath([-25, 0, -25], [25, 0, 25]);
		assert.ok(path.length >= 6);
		assert.ok(Math.abs(path.at(-3) - 25) < 0.5);

		const built = await native.buildNavMesh(vertices, triangles, { tileSize: 32 });
		assert.equal(built.findPath([-25, 0, -25], [25, 0, 25]).length, path.length);
		assert.deepEqual(built.serialize().length, baked.length);

		const crowd = navMesh.createCrowd(16, 0.6);
		const positions = crowd.positions();
		const agents = [-10, 0, 10].map((z) => crowd.addAgent([-20, 0, z]));
		agents.forEach((agent, i) => crowd.requestMove(agent, [20, 0, (i - 1) * 10]));
		for (let i = 0; i < 450; ++i)
			crowd.update(1 / 30);
		assert.equal(crowd.positions(), positions, 'the view is reused, update() rewrites it in place');
		assert.ok(Number.isNaN(positions[15 * 3]), 'empty slots are NaN');
		agents.forEach((agent) => assert.ok(Math.abs(positions[agent * 3] - 20) < 1, `agent ${agent} reached the target`));
		crowd.destroy();
		navMesh.destroy();
		built.destroy();
	});

	it('builds ozz assets (tools) and samples characters in parallel (runtime)', () => {
		const skeletonBytes = tools.buildSkeleton([
			{ parent: -1, name: 'root' },
			{ parent: 0, translation: [0, 1, 0], name: 'child' },
		]);
		const s = Math.SQRT1_2;
		const animationBytes = tools.buildAnimation(1, [
			{ translations: [[0, 0, 0, 0]], rotations: [[0, 0, 0, 0, 1], [1, 0, 0, s, s]], scales: [[0, 1, 1, 1]] },
			{ translations: [[0, 0, 1, 0]], rotations: [[0, 0, 0, 0, 1]], scales: [[0, 1, 1, 1]] },
		]);

		const skeleton = native.loadSkeleton(skeletonBytes);
		const animation = native.loadAnimation(animationBytes);
		assert.equal(skeleton.jointCount, 2);
		assert.equal(animation.duration, 1);

		const instances = Array.from({ length: 64 }, () => skeleton.createInstance());
		native.sampleAnimations(instances.map((instance) => ({ instance, animation, ratio: 1 })));
		// At the end the root is rotated 90 degrees around Z: the child at local (0, 1, 0) ends up at (-1, 0, 0).
		for (const instance of instances) {
			const child = instance.modelMatrices().subarray(16, 32);
			assert.ok(Math.abs(child[12] + 1) < 1e-3 && Math.abs(child[13]) < 1e-3, `child translation ${child[12]}, ${child[13]}`);
		}
		instances.forEach((instance) => instance.destroy());
		animation.destroy();
		skeleton.destroy();
	});

	it('imports FBX and glTF, then runs the geometry tools on the result', async () => {
		const scene = await tools.loadScene(await readFile(path.join(testData, 'cube.fbx')));
		assert.equal(scene.meshCount, 1);
		const mesh = scene.mesh(0);
		scene.destroy();
		assert.equal(mesh.indices.length, 36);

		const optimized = tools.optimizeMesh(mesh.indices, mesh.positions);
		assert.equal(optimized.length, 36);
		const simplified = tools.simplifyMesh(mesh.indices, mesh.positions, 12, 0.1);
		assert.ok(simplified.indices.length <= 36);

		const [unwrap, hulls] = await Promise.all([
			tools.unwrapUvs(mesh, 32),
			tools.convexDecompose(mesh, { maxHulls: 4, voxelResolution: 10000 }),
		]);
		assert.ok(unwrap.width > 0 && unwrap.indices.length === 36);
		assert.ok(hulls.length >= 1 && hulls[0].vertices.length >= 12);

		if (mesh.uvs) {
			const tangents = tools.generateTangents(mesh);
			assert.equal(tangents.length, mesh.indices.length * 4);
		}
	});

	it('generates MSDF glyphs in parallel', async (t) => {
		let font;
		try {
			font = await readFile(fontPath);
		} catch {
			t.skip(`no font at ${fontPath}`);
			return;
		}
		const glyphs = await tools.msdfGlyphs(font, [...'Stride 引擎'].map((c) => c.codePointAt(0)), 32, 4);
		const letter = glyphs[0];
		assert.ok(letter.width > 10 && letter.pixels.length === letter.width * letter.height * 3);
		assert.equal(glyphs[6].pixels.length, 0); // space
	});
});
