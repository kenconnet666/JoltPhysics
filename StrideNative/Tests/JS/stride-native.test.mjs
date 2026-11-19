// End-to-end tests of both wasm modules in Node: tools produce assets, the runtime consumes them.
// 测的是打包出去的东西：npm 包 @stride-ts/native 的 JS/dist（Scripts/build-wasm.ps1 先构建它）。
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { after, before, describe, it } from 'node:test';
import { fileURLToPath, pathToFileURL } from 'node:url';
import path from 'node:path';
import { crc32, deflateSync } from 'node:zlib';

const dist = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', '..', 'JS', 'dist');
const { createStrideNative, NativeError, AnimationBatch } = await import(pathToFileURL(path.join(dist, 'stride-native.js')));
const { createStrideTools } = await import(pathToFileURL(path.join(dist, 'stride-tools.js')));
const [{ physics }, { texture }, { navigation }] = await Promise.all(['physics', 'texture', 'navigation'].map((name) => import(pathToFileURL(path.join(dist, `${name}.js`)))));
const features = [physics, texture, navigation];

// 示例插件不进 npm 包，从正式版的构建目录加载（Scripts/build-wasm.ps1 编出它）
const examplePlugin = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', '..', 'Build', 'WASM_Release', 'example-plugin.wasm');
const example = { name: 'example', url: pathToFileURL(examplePlugin) };

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

/** 最小的 PNG 编码器（RGBA8，每行滤波类型 None），测试解码用。 */
function encodePng(width, height, rgba) {
	const raw = Buffer.alloc((width * 4 + 1) * height);
	for (let y = 0; y < height; ++y) Buffer.from(rgba.buffer, y * width * 4, width * 4).copy(raw, y * (width * 4 + 1) + 1);
	const chunk = (type, data) => {
		const b = Buffer.alloc(12 + data.length);
		b.writeUInt32BE(data.length, 0);
		b.write(type, 4);
		data.copy(b, 8);
		b.writeUInt32BE(crc32(b.subarray(4, 8 + data.length)), 8 + data.length);
		return b;
	};
	const header = Buffer.alloc(13);
	header.writeUInt32BE(width, 0);
	header.writeUInt32BE(height, 4);
	header[8] = 8; // 每通道 8 位
	header[9] = 6; // RGBA
	return new Uint8Array(Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', header), chunk('IDAT', deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]));
}

describe('stride-native and stride-tools', () => {
	let native, tools;

	before(async () => {
		[native, tools] = await Promise.all([createStrideNative({ workerThreads: 4, features }), createStrideTools({ workerThreads: 4 })]);
	});

	after(() => {
		native?.shutdown();
		tools?.shutdown();
	});

	it('starts the requested worker threads', () => {
		assert.equal(native.workerThreadCount, 4);
		assert.ok(native.buffer instanceof SharedArrayBuffer);
	});

	it('loads the debug runtime (Jolt asserts, Emscripten checks) and steps physics with it', async () => {
		const debug = await createStrideNative({ debug: true, workerThreads: 2, features: [physics] });
		try {
			const world = debug.createPhysicsWorld(8);
			world.addBox({ position: [0, -1, 0], halfExtent: [10, 1, 10], motion: 'static' });
			world.addBox({ position: [0, 3, 0], halfExtent: [0.5, 0.5, 0.5] });
			for (let i = 0; i < 120; ++i)
				world.step(1 / 60);
			assert.ok(Math.abs(world.positions()[4] - 0.5) < 0.05, 'the box rests on the floor');
			world.destroy();
		} finally {
			debug.shutdown();
		}
	});

	it('loads only the requested features and explains how to load a missing one', async () => {
		const core = await createStrideNative({ workerThreads: 1 });
		try {
			assert.equal(core.hasFeature(physics), false);
			assert.throws(() => core.createPhysicsWorld(8), (error) => error instanceof NativeError && error.code === 'not-initialized'
				&& /import \{ physics \} from '@stride-ts\/native\/physics'/.test(error.message));
			await assert.rejects(core.transcodeKtx2(new Uint8Array(4)), /needs the texture feature/);
			assert.throws(() => core.loadNavMesh(new Uint8Array(4)), /needs the navigation feature/);
			// 核心自己的功能照常可用
			using transforms = core.createTransforms(4);
			transforms.allocate(2);
			assert.equal(transforms.slotLimit, 2);
		} finally {
			core.shutdown();
		}
		assert.equal(native.hasFeature(physics) && native.hasFeature('navigation'), true);
		await assert.rejects(createStrideNative({ workerThreads: 1, features: [{ ...physics, name: 'no-such' }] }), /must be a C identifier/);
		await assert.rejects(createStrideNative({ workerThreads: 1, features: [physics, physics] }), /listed twice/);
	});

	it('loads a third-party plugin that works on runtime data through the plugin C API', async () => {
		const host = await createStrideNative({ workerThreads: 2, features: [physics, example] });
		try {
			const plugin = host.plugin(example);
			assert.throws(() => native.plugin(example), /plugin 'example' is not loaded/);

			// 同步调用：在任务系统上并行改写变换存储的列
			using transforms = host.createTransforms(64);
			const slots = transforms.allocate(40);
			host.withArrays([Int32Array.from(slots)], (pointer) => host.check(plugin.sn_example_wave(host.handleOf(transforms), pointer, slots.length, 0.5, 2), 'sn_example_wave'));
			for (const slot of slots) assert.ok(Math.abs(transforms.position[slot * 3 + 1] - 2 * Math.sin(0.5 + slot)) < 1e-5);
			assert.throws(() => host.withArrays([Int32Array.of(99)], (pointer) => host.check(plugin.sn_example_wave(host.handleOf(transforms), pointer, 1, 0, 1), 'sn_example_wave')),
				/slot 99 is outside 1..64/);

			// 异步任务：和内置任务一样用 awaitTask 等
			const bytes = new Uint8Array(1000).map((_, i) => (i % 10 === 0 ? 7 : i % 5));
			const { most, counts, summary } = await host.withArraysAsync([bytes], (pointer) =>
				host.awaitTask(plugin.sn_example_histogram(pointer, bytes.length), 'sn_example_histogram', (task) => ({
					most: task.result(0),
					counts: task.output(Uint32Array, 0),
					summary: new TextDecoder().decode(task.output(Uint8Array, 1)),
				})));
			assert.equal(counts.length, 256);
			assert.equal(counts[7], 100);
			assert.equal(most, 1); // 0 只有 100 个（i % 10 === 5），1..4 各 200 个
			assert.equal(summary, '1000 bytes, most common 1');
			await assert.rejects(host.awaitTask(plugin.sn_example_histogram(0, 0), 'sn_example_histogram'), /no data/);

			// 插件自己的对象；sn_shutdown 时由插件注册的钩子释放
			const counter = host.withArrays([new TextEncoder().encode('hits\0')], (name) => host.check(plugin.sn_example_counter_create(name), 'create'));
			plugin.sn_example_counter_add(counter, 5);
			assert.equal(plugin.sn_example_counter_add(counter, 3), 8);
			assert.equal(host.module.UTF8ToString(plugin.sn_example_counter_name(counter)), 'hits');
			plugin.sn_example_counter_destroy(counter);
			assert.throws(() => host.check(plugin.sn_example_counter_add(counter, 1), 'sn_example_counter_add'), /unknown counter/);

			// 内置功能和插件可以一起用
			assert.ok(host.createPhysicsWorld(4) && host.hasFeature('example'));
		} finally {
			host.shutdown();
		}
		await assert.rejects(createStrideNative({ workerThreads: 1, features: [{ name: 'other', url: example.url }] }), /does not export sn_plugin_abi_other: declare it with SN_PLUGIN\(other\)/);
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

	it('collects debug counters, timings and memory only while collecting is on', () => {
		native.drainDebugLog();
		assert.equal(native.debugCollecting, false);
		native.debugCollecting = true;
		native.resetDebugTimings();
		try {
			using world = native.createPhysicsWorld(4);
			world.addBox({ position: [0, -1, 0], halfExtent: [5, 1, 5], motion: 'static' });
			world.step(1 / 60);
			world.step(1 / 60);
			const snapshot = native.debugSnapshot();
			assert.equal(snapshot.counters['physics.bodies'], 1);
			assert.equal(snapshot.timings['physics.step'].calls, 2);
			assert.ok(snapshot.memory.heapBytes > 0 && snapshot.memory.allocatedBytes > 0);
			native.resetDebugTimings();
			assert.equal(native.debugSnapshot().timings['physics.step'], undefined);
		} finally {
			native.debugCollecting = false;
		}
		assert.deepEqual(native.drainDebugLog(), []);
	});

	it('runs a Stride-style character on compound and mesh colliders, with layered queries', () => {
		// Third Person Platformer 的设置：重力 -40、阻尼 0.05；层 1（柱子）和层 2（角色）照常碰撞
		using transforms = native.createTransforms(4);
		const [playerSlot] = transforms.allocate(1);
		transforms.position.set([0, 0.05, 0], playerSlot * 3);
		transforms.update();
		const world = native.createPhysicsWorld(16, { transforms, gravity: [0, -40, 0], linearDamping: 0.05, angularDamping: 0.05 });
		try {
			// 地面是网格碰撞体（20 x 20，朝上），x = 5 有一面层 0 的墙，x = 8 有一根层 1 的柱子
			const mesh = world.createMesh(new Float32Array([-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10]), new Uint32Array([0, 2, 1, 0, 3, 2]));
			const [ground, wall, pole] = world.addBodies([
				{ shapes: [{ type: 'mesh', mesh }] },
				{ shapes: [{ type: 'box', size: [1, 4, 4], position: [0, 2, 0] }, { type: 'box', size: [1, 1, 1], position: [0, 5, 0] }], position: [5, 0, -6] },
				{ shapes: [{ type: 'cylinder', radius: 0.25, length: 4 }], position: [8, 2, 0], layer: 1 },
			]);
			assert.deepEqual([ground, wall, pole], [0, 1, 2]);
			const player = world.addBody({
				motion: 'character', transform: playerSlot, layer: 2, interpolated: true,
				shapes: [{ type: 'capsule', radius: 0.4, length: 1.2, position: [0, 0.95, 0] }],
				character: { jumpVelocity: 13 },
			});
			for (let i = 0; i < 30; i++) world.step(1 / 60);
			assert.equal(world.isGrounded(player), true);

			world.moveCharacter(player, 0, 4);
			for (let i = 0; i < 30; i++) world.step(1 / 60);
			const z = transforms.position[playerSlot * 3 + 2];
			assert.ok(z > 1.6 && z < 2.1, `walked to z = ${z}`);
			world.moveCharacter(player, 0, 0, true);
			world.step(1 / 60);
			world.step(1 / 60);
			assert.equal(world.isGrounded(player), false);
			const before = transforms.position[playerSlot * 3 + 1];
			world.step(1 / 60);
			world.interpolate(0.5);
			const halfway = transforms.position[playerSlot * 3 + 1];
			assert.ok(halfway > before && halfway < world.positions()[player * 3 + 1], 'interpolation writes a pose between the last two steps');

			// 角色也能瞬移、转身（Stride 的 CharacterComponent.Orientation），朝向一直保持；能读速度
			world.teleport(player, [0, 3, 2], [0, 1, 0, 0]);
			for (let i = 0; i < 5; i++) world.step(1 / 60);
			assert.ok(Math.abs(Math.abs(transforms.rotation[playerSlot * 4 + 1]) - 1) < 1e-3, 'the turn is kept');
			// 瞬移不改速度：还在起跳后的上升里（13 减去 8 步的重力）
			const velocity = [0, 0, 0];
			world.velocity(player, velocity, [0, 0, 0]);
			assert.ok(velocity[1] > 6 && velocity[1] < 9, `still rising at ${velocity[1]}`);

			// 射线：层 0 的墙在 4.5，只看层 1 时打到柱子的 7.75，角色在层 2
			const hit = world.rayCast([0, 2, -6], [1, 0, 0], 20, { mask: 1 << 0 });
			assert.ok(hit && Math.abs(hit.distance - 4.5) < 1e-3 && hit.body === wall && hit.normal[0] < -0.99);
			assert.ok(Math.abs(world.rayCast([0, 2, 0], [1, 0, 0], 20, { mask: 1 << 1 }).distance - 7.75) < 1e-3);
			assert.equal(world.rayCast([0, 2, 0], [1, 0, 0], 20, { mask: 1 << 3 }), undefined);
			const reused = world.sphereCast(0.5, [0, 2, -6], [1, 0, 0], 20);
			assert.ok(Math.abs(reused.distance - 4) < 0.01);
			assert.equal(world.sphereCast(0.5, [0, 2, -6], [1, 0, 0], 20, { mask: 1 << 0, hit: reused }), reused, 'the hit object is reused');
			// 起点在墙里：默认是距离 0 的命中，忽略起点重叠时打到后面的柱子
			assert.equal(world.sphereCast(0.5, [5, 2, -6], [1, 0, 0], 20).distance, 0);
			assert.equal(world.sphereCast(0.5, [5, 2, -6], [1, 0, 0], 20, { ignoreInitialOverlap: true }), undefined, 'nothing behind the wall on this line');
			assert.throws(() => world.moveCharacter(wall, 1, 0), /not a character/);
			assert.throws(() => world.addBody({ motion: 'dynamic', shapes: [{ type: 'mesh', mesh }, { type: 'sphere', radius: 1 }] }), /mesh shape must have only that shape/);
			assert.throws(() => native.createPhysicsWorld(4, { collisionMasks: [1, 2] }), /32 entries/);
		} finally {
			world.destroy();
		}
	});

	it('controls BodyComponent-style bodies: convex hulls, kinematic switching, teleport, velocity, impulse, gravity', () => {
		using transforms = native.createTransforms(4);
		const [slot] = transforms.allocate(1);
		transforms.position.set([0, 3, 0], slot * 3);
		transforms.update();
		const world = native.createPhysicsWorld(8, { transforms });
		try {
			world.addBody({ shapes: [{ type: 'box', size: [20, 1, 20] }], position: [0, -0.5, 0] });
			// 单位立方体的 8 个角
			const hull = world.createHull(new Float32Array([0, 1, 2, 3, 4, 5, 6, 7].flatMap((i) => [i & 1 ? 0.5 : -0.5, i & 2 ? 0.5 : -0.5, i & 4 ? 0.5 : -0.5])));
			const crate = world.addBody({ motion: 'dynamic', shapes: [{ type: 'convexHull', hull }], transform: slot, neverSleep: true });
			for (let i = 0; i < 120; i++) world.step(1 / 60);
			assert.ok(Math.abs(transforms.position[slot * 3 + 1] - 0.5) < 0.03, 'the hull rests on the floor');
			assert.equal(world.isAwake(crate), true);

			world.applyImpulse(crate, [0, 5, 0]);
			const linear = [0, 0, 0], angular = [0, 0, 0];
			world.velocity(crate, linear, angular);
			assert.ok(Math.abs(linear[1] - 5) < 1e-4);

			// 运动学：瞬移立即写回变换；按速度走，不受重力
			world.setKinematic(crate, true);
			world.teleport(crate, [0, 10, 0], [0, 0, 0, 1]);
			assert.equal(transforms.position[slot * 3 + 1], 10);
			world.setVelocity(crate, [1, 0, 0], [0, 0, 0]);
			for (let i = 0; i < 60; i++) world.step(1 / 60);
			assert.ok(Math.abs(transforms.position[slot * 3] - 1) < 0.01 && Math.abs(transforms.position[slot * 3 + 1] - 10) < 1e-4);

			// 动态、不受重力：不动；打开重力就落
			world.setKinematic(crate, false);
			world.setVelocity(crate, [0, 0, 0], [0, 0, 0]);
			world.setGravity(crate, false);
			for (let i = 0; i < 30; i++) world.step(1 / 60);
			assert.ok(Math.abs(transforms.position[slot * 3 + 1] - 10) < 1e-4);
			world.setGravity(crate, true);
			for (let i = 0; i < 30; i++) world.step(1 / 60);
			assert.ok(transforms.position[slot * 3 + 1] < 9);
			// 运行中改重力：朝上，下落的箱子（约 -4.9 m/s）一秒后往上走
			world.setWorldGravity([0, 9.8, 0]);
			for (let i = 0; i < 60; i++) world.step(1 / 60);
			const rising = [0, 0, 0];
			world.velocity(crate, rising, [0, 0, 0]);
			assert.ok(rising[1] > 0, 'gravity changed at run time');
			world.setWorldGravity([0, -9.8, 0]);
			world.setNeverSleep(crate, false);
			for (let i = 0; i < 240; i++) world.step(1 / 60);
			assert.equal(world.isAwake(crate), false, 'resting bodies fall asleep once allowed');
			world.setNeverSleep(crate, true);
			assert.equal(world.isAwake(crate), true);
			assert.throws(() => world.setKinematic(0, true), /not a dynamic or kinematic body/);

			// 空碰撞体（Stride 的 EmptyCollider）：不碰撞，穿过地面往下掉；不能放进复合形状
			const ghost = world.addBody({ motion: 'dynamic', shapes: [{ type: 'empty' }], position: [3, 1, 0], neverSleep: true });
			for (let i = 0; i < 60; i++) world.step(1 / 60);
			const ghostVelocity = [0, 0, 0];
			world.velocity(ghost, ghostVelocity, [0, 0, 0]);
			assert.ok(ghostVelocity[1] < -9, 'an empty shape does not collide');
			assert.throws(() => world.addBody({ motion: 'dynamic', shapes: [{ type: 'empty' }, { type: 'sphere', radius: 1 }] }), /empty shape cannot be part of a compound/);
		} finally {
			world.destroy();
		}
	});

	it('reports trigger and contact events per body pair, and removes bodies', () => {
		const world = native.createPhysicsWorld(8);
		try {
			const [ground, trigger, box] = world.addBodies([
				{ shapes: [{ type: 'box', size: [20, 1, 20] }], position: [0, -0.5, 0], contactEvents: true },
				{ shapes: [{ type: 'box', size: [4, 2, 4] }], position: [0, 3, 0], sensor: true, contactEvents: true },
				{ motion: 'dynamic', shapes: [{ type: 'box', size: [1, 1, 1] }], position: [0, 8, 0] },
			]);
			const seen = [];
			for (let i = 0; i < 180; i++) {
				world.step(1 / 60);
				seen.push(...world.contactEvents());
			}
			assert.deepEqual(seen, [
				{ type: 'started', self: trigger, other: box, depth: 0 },
				{ type: 'stopped', self: trigger, other: box, depth: 0 },
				{ type: 'started', self: ground, other: box, depth: 0 },
			]);
			world.removeBodies([box, box]);
			world.step(1 / 60);
			assert.deepEqual(world.contactEvents(), []);
			assert.throws(() => world.setSensor(box, false), /does not exist/);
			// 移除的编号被复用：同时存在的物体不超过容量，来回加、删不会用完
			const count = world.bodyCount;
			const [reused, appended] = world.addBodies([
				{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.5 }], position: [0, 5, 0] },
				{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.5 }], position: [2, 5, 0] },
			]);
			assert.deepEqual([reused, appended], [box, count]);
			assert.equal(world.positions()[appended * 3], 2);
			for (let i = 0; i < 20; i++) world.removeBodies(world.addBodies([{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.5 }] }]));
			// 第一次没有空闲编号，接在后面用掉一个；之后都复用它
			assert.equal(world.bodyCount, count + 2);
		} finally {
			world.destroy();
		}
	});

	it('reports the contact depth every step while touching when asked (Stride IContactHandler.OnTouching)', () => {
		const world = native.createPhysicsWorld(8);
		try {
			// 盒子落进一个 4 x 2 x 4 的传感器：穿过时每步报一个深度，先变深再变浅
			const trigger = world.addBody({ shapes: [{ type: 'box', size: [4, 2, 4] }], position: [0, 3, 0], sensor: true, contactEvents: 'touching' });
			const box = world.addBody({ motion: 'dynamic', shapes: [{ type: 'box', size: [1, 1, 1] }], position: [0, 6, 0] });
			const depths = [];
			const order = [];
			for (let i = 0; i < 90; i++) {
				world.step(1 / 60);
				for (const event of world.contactEvents()) {
					order.push(event.type);
					if (event.type === 'touching') {
						assert.deepEqual([event.self, event.other], [trigger, box]);
						depths.push(event.depth);
					}
				}
			}
			assert.equal(order[0], 'started');
			assert.equal(order.at(-1), 'stopped');
			assert.ok(depths.length >= 5, `${depths.length} touching events`);
			assert.ok(Math.max(...depths) > 0.5 && depths[0] < Math.max(...depths), `depths ${depths.map((d) => d.toFixed(2)).join(' ')}`);
			world.setContactEvents(trigger, false);
		} finally {
			world.destroy();
		}
	});

	it('adds Bepu-style constraints between bodies and removes them', () => {
		const world = native.createPhysicsWorld(8, { gravity: [0, -9, 0] });
		try {
			const [cube, sphere] = world.addBodies([
				{ motion: 'kinematic', shapes: [{ type: 'box', size: [2, 2, 2] }], position: [2, 0, -9] },
				{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 1 }], position: [-2, 0, -9], neverSleep: true },
			]);
			const constraint = world.addConstraint({ type: 'ballSocket', bodyA: cube, bodyB: sphere, localOffsetB: [4, 0, 0] });
			world.applyImpulse(sphere, [0, 0, 18]);
			for (let i = 0; i < 90; i++) world.step(1 / 60);
			const p = world.positions();
			assert.ok(Math.abs(Math.hypot(p[3] - p[0], p[4] - p[1], p[5] - p[2]) - 4) < 0.1, 'the sphere orbits 4 m from the cube');
			world.removeConstraint(constraint);
			world.removeConstraint(constraint);
			// 滑轨：在立方体的 X 轴上，离开不超过左边 4 米（检查范围字段写对了位置）
			world.teleport(sphere, [-2, 0, -9], [0, 0, 0, 1]);
			world.setVelocity(sphere, [0, 0, 0], [0, 0, 0]);
			world.addConstraint({ type: 'pointOnLine', bodyA: cube, bodyB: sphere, localDirection: [1, 0, 0] });
			const limit = world.addConstraint({ type: 'linearAxisLimit', bodyA: cube, bodyB: sphere, localAxis: [1, 0, 0], minimumOffset: -4, maximumOffset: 0 });
			world.applyImpulse(sphere, [-25, 0, 0]);
			for (let i = 0; i < 60; i++) world.step(1 / 60);
			const offset = world.positions()[3] - world.positions()[0];
			assert.ok(offset >= -4.05 && offset <= 0.05, `slider offset ${offset}`);
			assert.ok(Math.abs(world.positions()[4]) < 0.01, 'the sphere stays on the line');
			assert.ok(limit > constraint);
			assert.throws(() => world.addConstraint({ type: 'weld', bodyA: sphere, bodyB: sphere }), /two different bodies/);
			assert.throws(() => world.addConstraint({ type: 'spring', bodyA: cube, bodyB: sphere }), /unknown constraint type/);
		} finally {
			world.destroy();
		}
	});

	it('simulates a dynamic body with a closed mesh shape on a mesh floor (mass from the mesh, mesh-vs-mesh contacts, like Bepu)', () => {
		const world = native.createPhysicsWorld(8);
		try {
			// 地面也是网格：网格对网格的碰撞（Jolt 本来不支持）
			const floor = world.createMesh(new Float32Array([-10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10]), new Uint32Array([0, 2, 1, 0, 3, 2]));
			world.addBody({ motion: 'static', shapes: [{ type: 'mesh', mesh: floor }] });
			// 单位立方体的 12 个三角形（朝外）
			const corners = [0, 1, 2, 3, 4, 5, 6, 7].flatMap((i) => [i & 1 ? 0.5 : -0.5, i & 2 ? 0.5 : -0.5, i & 4 ? 0.5 : -0.5]);
			const faces = [0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3, 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5];
			const mesh = world.createMesh(new Float32Array(corners), new Uint32Array(faces));
			const cube = world.addBody({ motion: 'dynamic', shapes: [{ type: 'mesh', mesh, mass: 1 }], position: [0, 3, 0], neverSleep: true });
			for (let i = 0; i < 180; i++) world.step(1 / 60);
			assert.ok(Math.abs(world.positions()[cube * 3 + 1] - 0.5) < 0.05, `rests on the floor at ${world.positions()[cube * 3 + 1]}`);
			// 绕 Y 的转动惯量 m/6：冲量 1 得到角速度 6
			world.applyImpulse(cube, undefined, [0, 1, 0]);
			const angular = [0, 0, 0];
			world.velocity(cube, [0, 0, 0], angular);
			assert.ok(Math.abs(angular[1] - 6) < 0.1, `angular velocity ${angular}`);
		} finally {
			world.destroy();
		}
	});

	it('rejects bodies whose transform slot has a world matrix that is not a rotation (NaN poses hang the broad phase)', () => {
		using transforms = native.createTransforms(4);
		const [slot] = transforms.allocate(1);
		// 按 Stride 的公式，(0, 0.5, 0.5, 0) 得到的矩阵是奇异的
		transforms.rotation.set([0, 0.5, 0.5, 0], slot * 4);
		transforms.update();
		const world = native.createPhysicsWorld(4, { transforms });
		try {
			assert.throws(() => world.addBodies([{ motion: 'dynamic', shapes: [{ type: 'box', size: [1, 1, 1] }], transform: slot }]), /not a rotation/);
		} finally {
			world.destroy();
		}
	});

	it('steps a large-capacity world (the temp allocator falls back to malloc instead of aborting)', () => {
		const world = native.createPhysicsWorld(32768);
		try {
			world.addBodies([{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.5 }], position: [0, 5, 0] }]);
			world.step(1 / 60);
			assert.ok(world.positions()[1] < 5);
		} finally {
			world.destroy();
		}
	});

	it('drives single bodies toward world targets (servos, angular motor) and limits distances with a spring', () => {
		using transforms = native.createTransforms(4);
		const [slot] = transforms.allocate(1);
		transforms.update();
		const world = native.createPhysicsWorld(8, { gravity: [0, -9, 0], transforms });
		try {
			const [anchor, crate, wheel, ball] = world.addBodies([
				{ motion: 'kinematic', shapes: [{ type: 'box', size: [1, 1, 1] }], position: [0, 10, 0] },
				{ motion: 'dynamic', shapes: [{ type: 'box', size: [1, 1, 1] }], transform: slot, neverSleep: true },
				{ motion: 'dynamic', shapes: [{ type: 'cylinder', radius: 1, length: 0.5 }], position: [10, 0, 0], neverSleep: true, gravity: false },
				{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.5 }], position: [0, 4, 0], neverSleep: true },
			]);
			// 重力枪：线性伺服把箱子的中心拉到 (2, 3, 0)，角度伺服把它转到绕 Y 90°
			world.addConstraint({ type: 'oneBodyLinearServo', bodyA: crate, target: [2, 3, 0] });
			const quarter = [0, Math.SQRT1_2, 0, Math.SQRT1_2];
			world.addConstraint({ type: 'oneBodyAngularServo', bodyA: crate, targetOrientation: quarter });
			// 单体角马达：轮子绕世界 Y 每秒 2 弧度
			world.addConstraint({ type: 'oneBodyAngularMotor', bodyA: wheel, targetVelocity: [0, 2, 0] });
			// 距离限制：球离锚点不超过 3 米（软的，允许一点点超出）
			world.addConstraint({ type: 'distanceLimit', bodyA: anchor, bodyB: ball, minimumDistance: 0, maximumDistance: 3 });
			for (let i = 0; i < 180; i++) world.step(1 / 60);
			const p = world.positions(), r = transforms.rotation.subarray(slot * 4, slot * 4 + 4);
			assert.ok(Math.hypot(p[3] - 2, p[4] - 3, p[5]) < 0.1, `servo position ${[p[3], p[4], p[5]]}`);
			assert.ok(Math.abs(Math.abs(r[1]) - Math.SQRT1_2) < 0.02, `servo orientation ${[...r]}`);
			const angular = [0, 0, 0];
			world.velocity(wheel, [0, 0, 0], angular);
			assert.ok(Math.abs(angular[1] - 2) < 0.05 && Math.abs(angular[0]) < 0.05, `motor ${angular}`);
			assert.ok(Math.hypot(p[9], p[10] - 10, p[11]) < 3.2, `distance ${p[10]}`);
			assert.throws(() => world.addConstraint({ type: 'oneBodyLinearServo', bodyA: 99, target: [0, 0, 0] }), /does not exist|99/);
		} finally {
			world.destroy();
		}
	});

	it('culls instances against view frustums in native memory, batch by batch', () => {
		using transforms = native.createTransforms(8);
		const slots = transforms.allocate(4);
		// 盒子形状的视锥：|x| <= 5、|y| <= 5、-20 <= z <= 0；第 0 批是单位立方体，第 1 批的包围盒半长是 0（总是可见）
		const at = [[-10, 0, -5], [0, 0, 3], [4.8, 0, -10], [100, 0, 0]];
		slots.forEach((slot, i) => transforms.position.set(at[i], slot * 3));
		transforms.update();
		using culler = transforms.createCuller();
		culler.reserve(4, 2);
		culler.slots.set(slots);
		culler.batchEnds.set([3, 4]);
		culler.bounds.set([0, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0, 0]);
		const planes = [1, 0, 0, 5, -1, 0, 0, 5, 0, 1, 0, 5, 0, -1, 0, 5, 0, 0, -1, 0, 0, 0, 1, 20];
		culler.setView(0, planes, 6, [0, 0, -1, 0]);
		culler.setView(1, planes, 4);
		culler.cull(4, 2, 2);
		assert.deepEqual(Array.from(culler.counts.subarray(0, 4)), [1, 1, 2, 1]);
		assert.equal(culler.visibleSlots[0], slots[2]);
		assert.equal(culler.visibleSlots[3], slots[3]);
		assert.deepEqual(Array.from(culler.visibleSlots.subarray(4, 6)), [slots[1], slots[2]]);
		assert.ok(Math.abs(culler.minDistance(0) - 9.5) < 1e-5 && Math.abs(culler.maxDistance(0) - 10.5) < 1e-5);
		assert.equal(culler.minDistance(1), Infinity, 'the second view does not measure distances');
		assert.throws(() => culler.setView(8, planes, 6), /view index must be 0..7/);
		// 容量不够时重新分配
		culler.reserve(100, 40);
		assert.ok(culler.slotCapacity >= 100 && culler.batchCapacity >= 40);
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
		const near = (actual, expected, epsilon = 1e-4) => assert.ok(Math.abs(actual - expected) < epsilon, `${actual} is not near ${expected}`);
		// 只算子节点这一条链（父节点也算），别的槽位不动
		transforms.updateSlot(child);
		[12, 4, 0].forEach((v, i) => near(world[child * 16 + 12 + i], v));
		near(world[parent * 16 + 12], 10);
		near(world[root * 16 + 13], 0);
		transforms.update();
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

	it('links a slot to another slot through a matrix (Stride TransformLink, ModelNodeLink), ignoring its parent', () => {
		using transforms = native.createTransforms(8);
		const [model, hand, other] = transforms.allocate(3);
		const { position, world } = transforms;
		const near = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-4, `${actual} is not near ${expected}`);
		// 模型在 (10, 0, 0)；骨骼节点在模型空间 (0, 2, 0)；手的本地平移 (1, 0, 0)；手的父节点是 other（链接后不看）
		position[model * 3] = 10;
		position[other * 3 + 2] = 50;
		transforms.setParent(hand, other);
		position[hand * 3] = 1;
		const node = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 2, 0, 1];
		transforms.setLink(hand, model, node);
		transforms.update();
		[11, 2, 0].forEach((v, i) => near(world[hand * 16 + 12 + i], v));
		// 每帧换节点矩阵；只算这条链也一样
		node[13] = 3;
		transforms.setLink(hand, model, node);
		transforms.updateSlot(hand);
		[11, 3, 0].forEach((v, i) => near(world[hand * 16 + 12 + i], v));
		// 成环的链接拒绝；链接到的槽位释放后，回到按父节点算
		assert.throws(() => transforms.setLink(model, hand, node), /cannot link to its own descendant/);
		assert.throws(() => transforms.setParent(model, hand), /cannot become the parent of a slot linked to it/);
		transforms.free(model);
		transforms.update();
		[1, 0, 50].forEach((v, i) => near(world[hand * 16 + 12 + i], v));
		transforms.setLink(hand, null);
	});

	it('releases native objects with using and reports failures as NativeError codes', async () => {
		let escaped;
		{
			using world = native.createPhysicsWorld(4);
			escaped = world;
			assert.equal(world.destroyed, false);
		}
		assert.equal(escaped.destroyed, true);
		assert.throws(() => native.loadNavMesh(new Uint8Array(16)), (error) => error instanceof NativeError && error.code === 'failed' && error.status === -3);
		await assert.rejects(native.transcodeKtx2(new Uint8Array(4), 'png'), /target must be one of 'rgba8', 'bc7', 'astc4x4', 'etc2rgba8', not 'png'/);
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

	it('resizes RGBA8 with Lanczos3: a constant image stays constant, sizes are checked', async () => {
		const rgba = new Uint8Array(7 * 5 * 4);
		for (let i = 0; i < 7 * 5; ++i)
			rgba.set([200, 100, 50, 255], i * 4);
		const resized = await tools.resizeImage(rgba, 7, 5, 8, 8);
		assert.equal(resized.length, 8 * 8 * 4);
		for (let i = 0; i < 8 * 8; ++i)
			assert.deepEqual([...resized.subarray(i * 4, i * 4 + 4)], [200, 100, 50, 255]);
		await assert.rejects(async () => tools.resizeImage(rgba, 7, 5, 0, 8), /both sizes are required/);
	});

	it('decodes PNG and encodes KTX2 (tools), then transcodes every mip level in parallel (runtime)', async () => {
		const size = 128;
		const rgba = new Uint8Array(size * size * 4);
		for (let i = 0; i < size * size; ++i)
			rgba.set([(i % size) * 2, Math.floor(i / size) * 2, 100, 255], i * 4);
		const decoded = await tools.decodeImage(encodePng(size, size, rgba));
		assert.deepEqual([decoded.width, decoded.height, decoded.channels], [size, size, 4]);
		assert.deepEqual(decoded.pixels, rgba);
		await assert.rejects(tools.decodeImage(new Uint8Array(8)), /image decode failed/);
		const ktx2 = await tools.encodeBasis(decoded.pixels, size, size, { mode: 'uastc', mipmaps: true });
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

	it('prefilters a cubemap for GGX and projects it onto spherical harmonics (tools)', async () => {
		const size = 16;
		const faces = new Float32Array(6 * size * size * 4);
		for (let i = 0; i < faces.length; i += 4) faces.set([0.25, 0.5, 1, 1], i);
		const prefiltered = await tools.prefilterCubemapGGX({ size, faces }, 64, 32);
		assert.deepEqual(prefiltered.levels.map((l) => l.length), [64, 32, 16, 8, 4, 2, 1].map((e) => 6 * e * e * 4));
		assert.ok(Math.abs(prefiltered.levels[3][1] - 0.5) < 0.01);
		const sh = await tools.cubemapSH({ size, faces });
		assert.equal(sh.length, 27);
		assert.ok(Math.abs(sh[0] - 0.25) < 0.01 && Math.abs(sh[2] - 1) < 0.01 && Math.abs(sh[5]) < 0.01);
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

	it('builds a navmesh at runtime from the static bodies of a physics world (Stride RecastMeshSystem)', async () => {
		const world = native.createPhysicsWorld(16);
		try {
			// 地面 20 x 20（层 6），中间一堵墙（层 6）；同一处一个触发器（层 6）和一个层 0 的箱子都不算
			// 地面用带本地偏移的盒子（复合形状里的叶子，TopDownRPG 的 GridBase10x10）
			world.addBody({ shapes: [{ type: 'box', size: [20, 1, 20], position: [0, -0.5, 0] }], layer: 6 });
			world.addBody({ shapes: [{ type: 'box', size: [1, 2, 16] }], position: [0, 1, -2], layer: 6 });
			world.addBody({ shapes: [{ type: 'box', size: [4, 4, 4] }], position: [0, 2, 8], layer: 6, sensor: true });
			world.addBody({ shapes: [{ type: 'box', size: [4, 4, 4] }], position: [0, 2, 8], layer: 0 });
			world.addBody({ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 1 }], position: [5, 5, 5], layer: 6 });
			const triangles = world.staticTriangles(1 << 6);
			assert.equal(triangles.length % 9, 0);
			// 两个盒子，每个面至少两个三角形
			assert.ok(triangles.length / 9 >= 24, `${triangles.length / 9} triangles`);
			const ys = triangles.filter((_, i) => i % 3 === 1);
			assert.ok(Math.max(...ys) < 2.01, 'the sensor and the layer-0 box are left out');
			const navMesh = await native.buildNavMesh(triangles, Int32Array.from(triangles.subarray(0, triangles.length / 3), (_, i) => i),
				{ cellSize: 0.1, cellHeight: 0.1, agentRadius: 0.5, agentHeight: 1.2, tileSize: 32 });
			// 墙从 z = -10 挡到 z = 6：从墙左边到右边要绕过 z = 6 那头
			const path = navMesh.findPath([-3, 0, -5], [3, 0, -5]);
			assert.ok(path.length >= 9, `${path.length / 3} corners`);
			assert.ok(Math.max(...path.filter((_, i) => i % 3 === 2)) > 6, 'the path goes around the end of the wall');
			navMesh.destroy();
		} finally {
			world.destroy();
		}
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
		using batch = new AnimationBatch(native);
		for (const instance of instances) batch.instance(instance).push(animation, 1);
		batch.evaluate();
		// At the end the root is rotated 90 degrees around Z: the child at local (0, 1, 0) ends up at (-1, 0, 0).
		for (const instance of instances) {
			const child = instance.modelMatrices().subarray(16, 32);
			assert.ok(Math.abs(child[12] + 1) < 1e-3 && Math.abs(child[13]) < 1e-3, `child translation ${child[12]}, ${child[13]}`);
		}

		// 混合（同 Stride 的 AnimationBlender）：0 秒是 0°、1 秒是 90°，各取一份按 0.5 混合是 45°；再叠加一份 1 秒的姿势（加 90°）是 135°
		const [a] = instances;
		const childAngle = () => {
			const child = a.modelMatrices().subarray(16, 32);
			// + 0：-0 换成 0（strictEqual 区分它们）
			return Math.round(Math.atan2(-child[12], child[13]) * 180 / Math.PI) + 0;
		};
		batch.clear().instance(a).push(animation, 0).push(animation, 1).blend(0.5).evaluate();
		assert.equal(childAngle(), 45);
		batch.clear().instance(a).push(animation, 0.5).push(animation, 1).add(1).evaluate();
		assert.equal(childAngle(), 135);
		batch.clear().instance(a).push(animation, 1).push(animation, 1).subtract(1).evaluate();
		assert.equal(childAngle(), 0);
		batch.clear().instance(a).evaluate();
		assert.equal(childAngle(), 0, 'no operations: rest pose');
		assert.throws(() => batch.clear().instance(a).push(animation, 0).blend(0.5).evaluate(), /animation evaluation request 0 failed/);

		// 蒙皮矩阵 = 关节的模型矩阵 × 逆绑定；绑定时 child 在 (0, 1, 0)，逆绑定是平移 (0, -1, 0)
		const skin = native.createSkin(new Uint32Array([1]), new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, -1, 0, 1]));
		using palette = skin.createPalette();
		batch.clear().instance(a).push(animation, 1).skin(skin, palette).evaluate();
		// 绑定时在 child 原点的顶点 (0, 1, 0) 现在在 (-1, 0, 0)：蒙皮矩阵把 (0, 1, 0) 变到 (-1, 0, 0)
		const m = palette.matrices();
		assert.ok(Math.abs(m[4] * 1 + m[12] + 1) < 1e-3 && Math.abs(m[5] * 1 + m[13]) < 1e-3, `skinned (${m[4] + m[12]}, ${m[5] + m[13]})`);
		skin.destroy();
		instances.forEach((instance) => instance.destroy());
		animation.destroy();
		skeleton.destroy();
	});

	it('imports OBJ through ufbx when asked (no .mtl, material names from usemtl)', async () => {
		const obj = ['o Quad', 'v 0 0 0', 'v 1 0 0', 'v 1 1 0', 'v 0 1 0', 'vn 0 0 1', 'usemtl Red', 'f 1//1 2//1 3//1 4//1', ''].join('\n');
		const scene = await tools.loadScene(new TextEncoder().encode(obj), 'obj');
		assert.equal(scene.meshCount, 1);
		assert.deepEqual(scene.materials, ['Red']);
		const mesh = scene.mesh(0);
		scene.destroy();
		assert.equal(mesh.indices.length, 6);
		assert.equal(mesh.positions.length, 12);
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

	it('describes fonts and reads codepoints, kerning and quadratic outlines', async (t) => {
		let font;
		try {
			font = await readFile(fontPath);
		} catch {
			t.skip(`no font at ${fontPath}`);
			return;
		}
		const description = tools.describeFont(font);
		assert.equal(description.fontCount, 1);
		assert.ok(description.unitsPerEm > 0 && description.ascent > 0 && description.descent < 0);
		// 名字在 18 个整数之后（读错位置时会少掉前几个字母）
		if (fontPath.endsWith('arial.ttf')) assert.deepEqual([description.family, description.subfamily], ['Arial', 'Regular']);
		else assert.ok(description.family.length > 0);
		const { codepoints, glyphs, kerning } = await tools.fontCodepoints(font);
		assert.equal(codepoints.length, glyphs.length);
		assert.equal(kerning.length % 3, 0);
		const glyphOf = (c) => glyphs[codepoints.indexOf(c.codePointAt(0))];
		const [o, space] = await tools.glyphOutlines(font, [glyphOf('o'), glyphOf(' ')]);
		assert.ok(o.curves.length >= 8 * 6 && o.advance > 0);
		assert.equal(o.curves.length % 6, 0);
		assert.equal(space.curves.length, 0);
	});
});
