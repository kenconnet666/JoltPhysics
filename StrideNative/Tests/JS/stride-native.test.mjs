// End-to-end tests of both wasm modules in Node: tools produce assets, the runtime consumes them.
// 测的是打包出去的东西：npm 包 @stride-ts/native 的 JS/dist（Scripts/build-wasm.ps1 先构建它）。
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { after, before, describe, it } from 'node:test';
import { fileURLToPath, pathToFileURL } from 'node:url';
import path from 'node:path';
import { crc32, deflateSync } from 'node:zlib';

const dist = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', '..', 'JS', 'dist');
const { createStrideNative, NativeError, AnimationBatch, defaultWorkerThreads } = await import(pathToFileURL(path.join(dist, 'stride-native.js')));
const { createStrideTools } = await import(pathToFileURL(path.join(dist, 'stride-tools.js')));
const [{ physics }, { texture }, { navigation }] = await Promise.all(['physics', 'texture', 'navigation'].map((name) => import(pathToFileURL(path.join(dist, `${name}.js`)))));
const features = [physics, texture, navigation];

// 示例插件不进 npm 包，从正式版的构建目录加载（Scripts/build-wasm.ps1 编出它）
const examplePlugin = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', '..', 'Build', 'WASM_Release', 'example-plugin.wasm');
// 插件的导出里哪些位置是指针（模块是 64 位的）：返回值在前，'p' 是指针
const example = {
	name: 'example',
	url: pathToFileURL(examplePlugin),
	signatures: { sn_example_wave: '__p___', sn_example_histogram: '_p_', sn_example_counter_create: '_p', sn_example_counter_name: 'p_' },
};

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

	it('defaults to one worker per core minus the caller, at most 8', () => {
		assert.equal(defaultWorkerThreads(), Math.min(16, Math.max(1, navigator.hardwareConcurrency - 1)));
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

	it('updates a level with thousands of slots on several threads, same result as one by one', () => {
		const roots = 20000;
		using transforms = native.createTransforms(roots * 2);
		const slots = transforms.allocate(roots * 2);
		for (let i = 0; i < roots; i++) {
			transforms.position[slots[i] * 3] = i;
			transforms.position[slots[roots + i] * 3 + 1] = 1;
			transforms.setParent(slots[roots + i], slots[i]);
		}
		transforms.update();
		for (let i = 0; i < roots; i += 997) {
			assert.equal(transforms.world[slots[i] * 16 + 12], i);
			assert.equal(transforms.world[slots[roots + i] * 16 + 12], i);
			assert.equal(transforms.world[slots[roots + i] * 16 + 13], 1);
		}
		assert.equal(transforms.changedSlots().length, roots * 2);
		// 第二轮只动一个父节点：它和它的子节点变了，别的不报
		transforms.position[slots[12345] * 3 + 2] = 5;
		transforms.update();
		assert.deepEqual([...transforms.changedSlots()].sort((a, b) => a - b), [slots[12345], slots[roots + 12345]].sort((a, b) => a - b));
		assert.equal(transforms.world[slots[roots + 12345] * 16 + 14], 5);
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
			// 只改朝向（鼠标转视角带着角色转）：插值到一半的位置不动，旋转马上写回
			world.setRotation(player, [0, Math.SQRT1_2, 0, Math.SQRT1_2]);
			assert.equal(transforms.position[playerSlot * 3 + 1], halfway);
			assert.ok(Math.abs(transforms.rotation[playerSlot * 4 + 1] - Math.SQRT1_2) < 1e-4, 'the rotation is written back at once');

			// 角色也能瞬移、转身（Stride 的 CharacterComponent.Orientation），朝向一直保持；能读速度
			world.teleport(player, [0, 3, 2], [0, 1, 0, 0]);
			for (let i = 0; i < 5; i++) world.step(1 / 60);
			assert.ok(Math.abs(Math.abs(transforms.rotation[playerSlot * 4 + 1]) - 1) < 1e-3, 'the turn is kept');
			// 瞬移不改速度：还在起跳后的上升里（13 减去 8 步的重力）
			const velocity = [0, 0, 0];
			world.velocity(player, velocity, [0, 0, 0]);
			assert.ok(velocity[1] > 6 && velocity[1] < 9, `still rising at ${velocity[1]}`);
			// 速度也能设（回到出生点时清零）、能加冲量（击退）：角色逻辑从这个速度接着算
			world.setVelocity(player, [0, 0, 0], [0, 0, 0]);
			world.velocity(player, velocity, [0, 0, 0]);
			assert.deepEqual(velocity, [0, 0, 0]);
			world.applyImpulse(player, [0, 5, 0], [0, 0, 0]);
			world.velocity(player, velocity, [0, 0, 0]);
			assert.ok(velocity[1] > 0.5, `the impulse pushes the character up: ${velocity[1]}`);

			// 射线：层 0 的墙在 4.5，只看层 1 时打到柱子的 7.75，角色在层 2
			const hit = world.rayCast([0, 2, -6], [1, 0, 0], 20, { mask: 1 << 0 });
			assert.ok(hit && Math.abs(hit.distance - 4.5) < 1e-3 && hit.body === wall && hit.normal[0] < -0.99);
			assert.ok(Math.abs(world.rayCast([0, 2, 0], [1, 0, 0], 20, { mask: 1 << 1 }).distance - 7.75) < 1e-3);
			assert.equal(world.rayCast([0, 2, 0], [1, 0, 0], 20, { mask: 1 << 3 }), undefined);
			// 穿透的射线：穿过的每个物体各一个命中，从近到远。从墙的正上方往下：先是墙顶上的小盒子（复合碰撞体的两个盒子只算一次），再是地面
			const down = world.rayCastAll([5, 10, -6], [0, -1, 0], 20);
			assert.deepEqual(down.map((h) => h.body), [wall, ground]);
			assert.ok(Math.abs(down[0].distance - 4.5) < 1e-3 && Math.abs(down[1].distance - 10) < 1e-3, `distances ${down.map((h) => h.distance)}`);
			assert.ok(down[0].normal[1] > 0.99 && Math.abs(down[1].point[1]) < 1e-3);
			// 斜着穿过柱子再到地面；按层筛；容量不够时自己再查一次
			assert.deepEqual(world.rayCastAll([6.5, 2.5, 0], [1, -1, 0], 20).map((h) => h.body), [pole, ground]);
			assert.deepEqual(world.rayCastAll([6.5, 2.5, 0], [1, -1, 0], 20, { mask: 1 << 1 }).map((h) => h.body), [pole]);
			assert.deepEqual(world.rayCastAll([5, 10, -6], [0, -1, 0], 20, { capacity: 1 }).map((h) => h.body), [wall, ground]);
			assert.equal(world.rayCastAll([0, 2, 0], [1, 0, 0], 20, { mask: 1 << 3 }).length, 0);
			const reused = world.sphereCast(0.5, [0, 2, -6], [1, 0, 0], 20);
			assert.ok(Math.abs(reused.distance - 4) < 0.01);
			assert.equal(world.sphereCast(0.5, [0, 2, -6], [1, 0, 0], 20, { mask: 1 << 0, hit: reused }), reused, 'the hit object is reused');
			// 起点在墙里：默认是距离 0 的命中，忽略起点重叠时打到后面的柱子
			assert.equal(world.sphereCast(0.5, [5, 2, -6], [1, 0, 0], 20).distance, 0);
			assert.equal(world.sphereCast(0.5, [5, 2, -6], [1, 0, 0], 20, { ignoreInitialOverlap: true }), undefined, 'nothing behind the wall on this line');
			// 重叠查询：墙上的球、按遮罩、容量不够时再查一次放得下全部
			assert.deepEqual(world.overlap({ type: 'sphere', radius: 0.6, position: [5, 2, -6] }), [wall]);
			assert.deepEqual(world.overlap({ type: 'sphere', radius: 0.6, position: [5, 2, -6] }, { mask: 1 << 1 }), []);
			assert.deepEqual(world.overlap({ type: 'capsule', radius: 0.2, length: 1, position: [5, 2, -6], rotation: [0, 0, 0.7071068, 0.7071068] }, { capacity: 0 }), [wall]);
			assert.deepEqual(world.overlap({ type: 'sphere', radius: 0.4, position: [2, 2, -6] }), []);
			assert.throws(() => world.moveCharacter(wall, 1, 0), /not a character/);
			assert.throws(() => world.addBody({ motion: 'dynamic', shapes: [{ type: 'mesh', mesh }, { type: 'sphere', radius: 1 }] }), /mesh shape must have only that shape/);
			assert.throws(() => native.createPhysicsWorld(4, { collisionMasks: [1, 2] }), /32 entries/);
		} finally {
			world.destroy();
		}
	});

	it('drives a wheeled vehicle: throttle moves it along its forward axis, wheels rest on the ground, steering turns it', () => {
		const world = native.createPhysicsWorld(8);
		try {
			world.addBody({ shapes: [{ type: 'box', size: [400, 1, 400] }], position: [0, -0.5, 0] });
			const body = world.addBody({ motion: 'dynamic', shapes: [{ type: 'box', size: [1.8, 0.6, 4], mass: 1200 }], position: [0, 1, 0] });
			const wheel = (x, z, front) => ({ position: [x, -0.1, z], radius: 0.35, width: 0.25, suspensionMin: 0.1, suspensionMax: 0.4, maxSteerAngle: front ? 0.5 : 0, maxHandBrakeTorque: front ? 0 : 4000 });
			// 车头朝 -Z：前轴在 -Z 那头
			const car = world.addVehicle({ body, wheels: [wheel(-0.8, -1.4, true), wheel(0.8, -1.4, true), wheel(-0.8, 1.4, false), wheel(0.8, 1.4, false)], drive: 'rear' });
			for (let i = 0; i < 60; i++) world.step(1 / 60);
			let state = world.vehicleState(car, 4);
			assert.equal(state.grounded, 4, 'all four wheels touch the ground after settling');
			const z0 = world.positions()[body * 3 + 2];
			for (let i = 0; i < 180; i++) {
				world.vehicleInput(car, 1, 0);
				world.step(1 / 60);
			}
			state = world.vehicleState(car, 4);
			const z1 = world.positions()[body * 3 + 2];
			assert.ok(z1 < z0 - 5, `full throttle for 3 s drives it forward (-Z): z ${z0} -> ${z1}`);
			assert.ok(state.speed > 3 && state.gear >= 1 && state.rpm > 0, `moving forward in a forward gear: ${JSON.stringify({ speed: state.speed, gear: state.gear, rpm: state.rpm })}`);
			assert.ok(state.wheels[0].position[2] < 0 && state.wheels[2].position[2] > 0, 'wheel transforms are relative to the body');
			const x1 = world.positions()[body * 3];
			for (let i = 0; i < 120; i++) {
				world.vehicleInput(car, 0.5, 1);
				world.step(1 / 60);
			}
			assert.ok(world.positions()[body * 3] > x1 + 1, 'steering right turns it towards +X');
			assert.throws(() => world.addVehicle({ body, wheels: [wheel(0, 0, false)] }), /even number of wheels/);
			world.removeBodies([body]);
			assert.throws(() => world.vehicleInput(car, 1, 0), /does not exist/);
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

	it('sweeps continuous bodies so a fast small one does not pass through a thin wall', () => {
		const world = native.createPhysicsWorld(8);
		try {
			// 1 厘米厚的墙，2 厘米的球每步走 1 米：离散的穿过去，连续的停在墙前
			const [, discrete, continuous] = world.addBodies([
				{ shapes: [{ type: 'box', size: [0.01, 4, 4] }], position: [0, 0, 0] },
				{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.01 }], position: [-2.5, 0, -1], gravity: false },
				{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.01 }], position: [-2.5, 0, 1], gravity: false, continuous: true },
			]);
			world.setVelocity(discrete, [60, 0, 0]);
			world.setVelocity(continuous, [60, 0, 0]);
			for (let i = 0; i < 30; i++) world.step(1 / 60);
			// 穿过去的照旧往 +x 飞；挡住的速度没了（或者弹回来）
			const vx = (body) => {
				const linear = [0, 0, 0];
				world.velocity(body, linear, [0, 0, 0]);
				return linear[0];
			};
			assert.ok(vx(discrete) > 50, `the discrete sphere passes through (vx = ${vx(discrete)})`);
			assert.ok(vx(continuous) < 10, `the continuous sphere is stopped by the wall (vx = ${vx(continuous)})`);
		} finally {
			world.destroy();
		}
	});

	it('keeps contacts of sleeping bodies, and ends them when the body wakes up somewhere else', () => {
		const world = native.createPhysicsWorld(8);
		try {
			const [, trigger, box] = world.addBodies([
				{ shapes: [{ type: 'box', size: [20, 1, 20] }], position: [0, -0.5, 0] },
				{ shapes: [{ type: 'box', size: [4, 2, 4] }], position: [0, 1, 0], sensor: true, contactEvents: true },
				{ motion: 'dynamic', shapes: [{ type: 'box', size: [1, 1, 1] }], position: [0, 0.6, 0] },
			]);
			const run = (steps) => {
				const seen = [];
				for (let i = 0; i < steps; i++) {
					world.step(1 / 60);
					seen.push(...world.contactEvents().map((e) => e.type));
				}
				return seen;
			};
			// 在触发器里停下、睡着：接触还算着，不报结束（同 Bepu）
			assert.deepEqual(run(240), ['started']);
			assert.equal(world.isAwake(box), false);
			// 睡着时被瞬移出去（瞬移会叫醒它）：醒着两步后报结束；再瞬移回来又报开始
			world.teleport(box, [10, 0.5, 0], [0, 0, 0, 1]);
			assert.deepEqual(run(5), ['stopped']);
			world.teleport(box, [0, 0.5, 0], [0, 0, 0, 1]);
			assert.deepEqual(run(5), ['started']);
			// 睡着以后被叫醒、还在原地：Jolt 重新加入同一对，不重复报开始
			run(240);
			assert.equal(world.isAwake(box), false);
			world.setAwake(box, true);
			assert.deepEqual(run(5), []);
			assert.equal(trigger, 1);
		} finally {
			world.destroy();
		}
	});

	it('ends the contact when a sensor leaves a sleeping body (a coin flying past a player standing still)', () => {
		const world = native.createPhysicsWorld(8);
		try {
			// 玩家：地上的箱子，站着不动就睡下；金币的触发器：运动学的（TopDownRPG 的金币上就是这样）。Jolt 的触发器从不睡
			const [, player, coin] = world.addBodies([
				{ shapes: [{ type: 'box', size: [20, 1, 20] }], position: [0, -0.5, 0] },
				{ motion: 'dynamic', shapes: [{ type: 'box', size: [0.5, 0.5, 0.5] }], position: [0, 0.25, 0] },
				{ motion: 'kinematic', shapes: [{ type: 'sphere', radius: 0.5 }], position: [5, 0.5, 0], sensor: true, contactEvents: true },
			]);
			const run = (steps) => {
				const seen = [];
				for (let i = 0; i < steps; i++) {
					world.step(1 / 60);
					seen.push(...world.contactEvents().filter((e) => e.other === player || e.self === player).map((e) => e.type));
				}
				return seen;
			};
			run(240);
			assert.equal(world.isAwake(player), false, 'the player sleeps');
			// 金币飞过睡着的玩家：进来、出去都报（以前出去时玩家睡着，当成"睡下引起的"挂起，之后再碰到不报开始）
			world.teleport(coin, [0, 0.5, 0], [0, 0, 0, 1]);
			assert.deepEqual(run(3), ['started']);
			world.teleport(coin, [5, 0.5, 0], [0, 0, 0, 1]);
			assert.deepEqual(run(3), ['stopped']);
			world.teleport(coin, [0, 0.5, 0], [0, 0, 0, 1]);
			assert.deepEqual(run(3), ['started']);
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
			// 开始接触时带着碰撞前的接近速度：落进触发器时掉了 3.5 米、落地时掉了 7.5 米（√(2gh)，按步长差一点）
			assert.deepEqual(seen.map(({ speed, point, normal, ...event }) => event), [
				{ type: 'started', self: trigger, other: box, depth: 0 },
				{ type: 'stopped', self: trigger, other: box, depth: 0 },
				{ type: 'started', self: ground, other: box, depth: 0 },
			]);
			assert.ok(Math.abs(seen[0].speed - Math.sqrt(2 * 9.81 * 3.5)) < 0.4, `into the trigger at ${seen[0].speed} m/s`);
			assert.equal(seen[1].speed, 0);
			assert.ok(Math.abs(seen[2].speed - Math.sqrt(2 * 9.81 * 7.5)) < 0.4, `onto the ground at ${seen[2].speed} m/s`);
			// 落地的接触点在地面上（y 约 0、在箱子底下），法线从地面指向箱子（朝上）
			assert.ok(Math.abs(seen[2].point[1]) < 0.05 && Math.hypot(seen[2].point[0], seen[2].point[2]) < 0.75, `contact at ${seen[2].point}`);
			assert.ok(seen[2].normal[1] > 0.99, `normal ${seen[2].normal}`);
			assert.deepEqual(seen[1].point, [0, 0, 0]);
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

	it('simulates cloth as a soft body: pinned corners hold, the rest hangs; pinned vertices follow targets', () => {
		const world = native.createPhysicsWorld(8);
		try {
			// 1 米见方、11 × 11 个点的布，水平放在 3 米高，两个角钉住
			const side = 11, spacing = 0.1;
			const vertices = new Float32Array(side * side * 3), inverseMasses = new Float32Array(side * side).fill(1 / 0.01);
			for (let z = 0; z < side; z++) for (let x = 0; x < side; x++) vertices.set([x * spacing - 0.5, 0, z * spacing - 0.5], (z * side + x) * 3);
			inverseMasses[0] = 0;
			inverseMasses[side - 1] = 0;
			const indices = [];
			for (let z = 0; z + 1 < side; z++) for (let x = 0; x + 1 < side; x++) {
				const a = z * side + x;
				indices.push(a, a + side, a + side + 1, a, a + side + 1, a + 1);
			}
			const cloth = world.addSoftBody({ vertices, inverseMasses, indices: Uint32Array.from(indices), position: [0, 3, 0], compliance: 1e-5, shearCompliance: 1e-5, bendCompliance: 1e-3, longRange: 'geodesic', doubleSided: true });
			const out = new Float32Array(side * side * 8);
			let origin = [0, 0, 0];
			const at = (v) => [0, 1, 2].map((k) => origin[k] + out[v * 8 + k]);
			// 像单摆一样荡：对边的中间最低垂到钉子下面约 1 米（布长），离钉子不超过布的对角线（不伸长）
			let lowest = Infinity, farthest = 0;
			for (let i = 0; i < 240; i++) {
				world.step(1 / 60);
				origin = world.softBodyVertices(cloth, out, 8);
				const far = at(side * side - 1 - 5);
				lowest = Math.min(lowest, far[1]);
				farthest = Math.max(farthest, Math.hypot(far[0] + 0.5, far[1] - 3, far[2] + 0.5));
			}
			assert.ok(Math.hypot(...at(0).map((value, k) => value - [-0.5, 3, -0.5][k])) < 0.01, `pinned corner at ${at(0)}`);
			assert.ok(lowest < 2.1, `far edge hangs down to y = ${lowest}`);
			assert.ok(farthest < Math.hypot(0.5, 1) * 1.03, `cloth stretched to ${farthest}`);
			// 法线是单位向量
			for (let v = 0; v < side * side; v++) assert.ok(Math.abs(Math.hypot(out[v * 8 + 3], out[v * 8 + 4], out[v * 8 + 5]) - 1) < 1e-4);
			// 固定点按目标走：两个角花半秒往 +x 移 0.5 米（每步设一次目标）
			for (let i = 1; i <= 30; i++) {
				const dx = (0.5 * i) / 30;
				world.softBodyPin(cloth, Uint32Array.of(0, side - 1), Float64Array.of(-0.5 + dx, 3, -0.5, 0.5 + dx, 3, -0.5), 1 / 60);
				world.step(1 / 60);
			}
			origin = world.softBodyVertices(cloth, out, 8);
			assert.ok(Math.abs(at(0)[0]) < 0.01 && Math.abs(at(side - 1)[0] - 1) < 0.01, `pinned corners moved to ${at(0)} and ${at(side - 1)}`);
			// 风从 +z 吹：一秒以后整块布往 +z 飘
			const before = world.softBodyVertices(cloth, out, 8)[2] + out[(side * side - 6) * 8 + 2];
			for (let i = 0; i < 60; i++) {
				// 固定点的速度一直保持到下一次设：停在原地也要每步设（速度归零）
				world.softBodyPin(cloth, Uint32Array.of(0, side - 1), Float64Array.of(0, 3, -0.5, 1, 3, -0.5), 1 / 60);
				world.softBodyWind(cloth, [0, 0, 8], 0.6, 1 / 60);
				world.step(1 / 60);
			}
			const after = world.softBodyVertices(cloth, out, 8)[2] + out[(side * side - 6) * 8 + 2];
			assert.ok(after > before + 0.2, `wind blew the far edge from z = ${before} to ${after}`);
			assert.throws(() => world.softBodyVertices(0 === cloth ? 1 : 0, out, 8), /does not exist|not a soft body/);
			world.removeBodies([cloth]);
			assert.throws(() => world.softBodyVertices(cloth, out, 8), /does not exist/);
		} finally {
			world.destroy();
		}
	});

	it('freezes awake bodies far from the focus and wakes them when it comes back (physics level of detail)', () => {
		const world = native.createPhysicsWorld(8);
		try {
			const [near, far] = world.addBodies([
				{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.5 }], position: [0, 50, 0] },
				{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.5 }], position: [500, 50, 0] },
			]);
			world.setVelocity(near, [1, 0, 0]);
			world.setVelocity(far, [1, 0, 0]);
			world.step(1 / 60);
			assert.deepEqual(world.lod([0, 0, 0], 200, 150), { frozen: 1, slept: 1, woken: 0 });
			const before = world.positions().slice(0, 6);
			for (let i = 0; i < 30; i++) world.step(1 / 60);
			const after = world.positions();
			// 近的接着落，远的停在原处
			assert.ok(after[1] < before[1] - 0.5, `near body fell from ${before[1]} to ${after[1]}`);
			assert.equal(after[4], before[4]);
			assert.deepEqual(world.lod([0, 0, 0], 200, 150), { frozen: 1, slept: 0, woken: 0 });
			// 关注点挪过去：远的叫醒、接着落，近的冻上
			assert.deepEqual(world.lod([480, 0, 0], 200, 150), { frozen: 1, slept: 1, woken: 1 });
			for (let i = 0; i < 30; i++) world.step(1 / 60);
			assert.ok(world.positions()[4] < before[4] - 0.5, 'far body falls again after waking');
			assert.throws(() => world.lod([0, 0, 0], 100, 150), /wake_distance/);
		} finally {
			world.destroy();
		}
	});

	it('lets a fast enough hit through a body with a break speed (no collision response that step)', () => {
		const fall = (breakSpeed) => {
			const world = native.createPhysicsWorld(8);
			try {
				const [crate, rock] = world.addBodies([
					{ shapes: [{ type: 'box', size: [1, 1, 1] }], position: [0, 0.5, 0], contactEvents: true },
					{ motion: 'dynamic', shapes: [{ type: 'sphere', radius: 0.2 }], position: [0, 4, 0] },
				]);
				world.setBreakSpeed(crate, breakSpeed);
				let speed = 0, after;
				for (let i = 0; i < 120 && after === undefined; i++) {
					world.step(1 / 60);
					const started = world.contactEvents().find((e) => e.type === 'started' && e.other === rock);
					if (started) {
						speed = started.speed;
						const linear = [0, 0, 0], angular = [0, 0, 0];
						world.velocity(rock, linear, angular);
						after = linear[1];
					}
				}
				return { speed, after };
			} finally {
				world.destroy();
			}
		};
		// 不碎时这一步就被挡住（几乎停下）；破碎速度够低时照原速度穿进去
		const blocked = fall(0), through = fall(3);
		assert.ok(blocked.speed > 6 && through.speed > 6, `hit at ${blocked.speed}, ${through.speed} m/s`);
		assert.ok(Math.abs(blocked.after) < 2, `blocked: ${blocked.after} m/s after the hit`);
		assert.ok(through.after < -6, `through: ${through.after} m/s after the hit`);
		assert.ok(fall(50).after > -2, 'a break speed above the hit blocks it');
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
		culler.reserve(4, 2, 2);
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
		assert.throws(() => culler.setView(4, planes, 6), /view index must be 0..3 \(reserve more views\)/);
		// 容量不够时重新分配；视图数不设上限（阴影多的帧一次几十个）
		culler.reserve(100, 40, 20);
		assert.ok(culler.slotCapacity >= 100 && culler.batchCapacity >= 40 && culler.viewCapacity >= 20);
		culler.slots.set(slots);
		culler.batchEnds.set([3, 4]);
		culler.bounds.set([0, 0, 0, 0.5, 0.5, 0.5, 0, 0, 0, 0, 0, 0]);
		for (let v = 0; v < 20; v++) culler.setView(v, planes, 4);
		culler.cull(4, 2, 20);
		// 只看四个侧面（同上面的第 1 个视图）
		assert.deepEqual(Array.from(culler.counts.subarray(19 * 2, 20 * 2)), [2, 1]);
		assert.deepEqual(Array.from(culler.visibleSlots.subarray(19 * 4, 19 * 4 + 2)), [slots[1], slots[2]]);
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
		// 容量不够时自动增长，列换成新的视图
		const before = transforms.generation;
		assert.equal(transforms.allocate(6).length, 6);
		assert.ok(transforms.generation > before && transforms.capacity >= 14);
		assert.equal(transforms.allocate(5).length, 5);
	});

	it('keeps far positions in double precision and renders them relative to the render origin', () => {
		using transforms = native.createTransforms(4);
		using world = native.createPhysicsWorld(4, { transforms });
		const [far, child] = transforms.allocate(2);
		transforms.setParent(child, far);
		// 2 万公里外（单精度在那里一个刻度是 2 米）：毫米也留得住
		transforms.position.set([20000000.001, 0, -15000000.002], far * 3);
		transforms.position[child * 3] = 0.5;
		transforms.update();
		assert.ok(Math.abs(transforms.world[child * 16 + 12] - 20000000.501) < 1e-6);
		assert.ok(Math.abs(transforms.world[child * 16 + 14] + 15000000.002) < 1e-6);
		// 渲染原点放在附近：渲染用的矩阵是小数，精度在相机附近
		transforms.setRenderOrigin(20000000, 0, -15000000);
		assert.ok(transforms.changedSlots().length === 2, 'moving the render origin marks every slot changed');
		assert.ok(Math.abs(transforms.renderWorld[child * 16 + 12] - 0.501) < 1e-6);
		assert.ok(Math.abs(transforms.renderWorld[child * 16 + 14] + 0.002) < 1e-6);
		// 物理也是双精度：远处的盒子落在远处的地板上，射线打中它
		world.addBox({ halfExtent: [10, 0.5, 10], position: [20000000, -0.5, -15000000], motion: 'static' });
		const box = world.addBox({ halfExtent: [0.5, 0.5, 0.5], transform: child, motion: 'dynamic' });
		for (let i = 0; i < 120; i++) world.step(1 / 60);
		transforms.update();
		assert.ok(Math.abs(world.positions()[box * 3 + 1] - 0.5) < 0.05, 'rests on the far floor');
		assert.ok(Math.abs(world.positions()[box * 3] - 20000000.501) < 0.01, 'kept its far x to the centimeter');
		const hit = world.rayCast([20000000.501, 10, -15000000.002], [0, -1, 0], 20);
		assert.ok(hit && Math.abs(hit.point[1] - 1) < 0.05 && Math.abs(hit.point[0] - 20000000.501) < 1e-3);
	});

	it('builds BVHs for ray tracing: every triangle in one leaf, children inside their parents, box BVH orders every primitive', async () => {
		const count = 500, positions = new Float32Array(count * 9), indices = new Uint32Array(count * 3);
		let seed = 7;
		const random = () => ((seed = (seed * 1103515245 + 12345) >>> 0) / 4294967296);
		for (let t = 0; t < count; t++) {
			const cx = random() * 100, cy = random() * 10, cz = random() * 100;
			for (let k = 0; k < 3; k++) {
				positions.set([cx + random(), cy + random(), cz + random()], (t * 3 + k) * 3);
				indices[t * 3 + k] = t * 3 + k;
			}
		}
		const { nodes, triangles, nodeCount } = await native.buildTriangleBvh(positions, indices);
		const floats = new Float32Array(nodes.buffer, nodes.byteOffset, nodes.length);
		assert.equal(nodes.length, nodeCount * 32);
		// 宽节点：4 个子节点的盒子按分量排（最小 xyz、最大 xyz 各 4 个），子节点、图元数；空的子节点盒子是反的
		const box = (n, i) => [0, 1, 2, 3, 4, 5].map((k) => floats[n * 32 + k * 4 + i]);
		const seen = new Set();
		let reached = 1;
		for (let n = 0; n < nodeCount; n++) {
			for (let i = 0; i < 4; i++) {
				const [x0, y0, z0, x1, y1, z1] = box(n, i);
				if (x0 > x1) continue;
				const child = nodes[n * 32 + 24 + i], leafCount = nodes[n * 32 + 28 + i];
				if (leafCount === 0) {
					// 子节点的子节点都在这个盒子里
					reached++;
					for (let j = 0; j < 4; j++) {
						const inner = box(child, j);
						if (inner[0] > inner[3]) continue;
						assert.ok(inner[0] >= x0 && inner[1] >= y0 && inner[2] >= z0 && inner[3] <= x1 && inner[4] <= y1 && inner[5] <= z1, `node ${n} child ${i} contains its children`);
					}
					continue;
				}
				assert.ok(leafCount <= 4);
				for (let t = child; t < child + leafCount; t++) seen.add(new Uint32Array(triangles.buffer, triangles.byteOffset)[t * 12 + 3]);
			}
		}
		assert.equal(seen.size, count);
		// 每个节点都有人指着（没有孤立的）；4 叉比二叉少得多（二叉树大约 2 × 三角形数个节点）
		assert.equal(reached, nodeCount);
		assert.ok(nodeCount < count * 0.6, `${nodeCount} wide nodes for ${count} triangles`);
		// 三角形太多时先简化：10 米的地板上 2000 件 2 厘米的"商品"（每件是共一个角的两个小方块，24 个三角形连成一块），限到 2000 个；
		// 小的按中心分格、每格换成一个盒子（12 个三角形，一格一两件），地板（大块）原样留着
		const items = 2000, itemPositions = new Float32Array((items * 15 + 4) * 3), itemIndices = [];
		const corners = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]];
		const faces = [0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5];
		for (let c = 0; c < items; c++) {
			const x = (c % 50) / 5, y = Math.floor(c / 500) * 0.5, z = Math.floor(c / 50) % 10, base = c * 15;
			// 第一个方块 8 个角；第二个方块的第 0 个角是第一个的第 7 个（共用），另外 7 个接在后面
			corners.forEach(([a, b, d], k) => itemPositions.set([x + a * 0.02, y + b * 0.02, z + d * 0.02], (base + k) * 3));
			corners.slice(1).forEach(([a, b, d], k) => itemPositions.set([x + 0.02 + a * 0.02, y + 0.02 + b * 0.02, z + 0.02 + d * 0.02], (base + 8 + k) * 3));
			for (const f of faces) itemIndices.push(base + f);
			for (const f of faces) itemIndices.push(f === 0 ? base + 7 : base + 7 + f);
		}
		const floor = items * 15;
		itemPositions.set([0, -0.1, 0, 10, -0.1, 0, 0, -0.1, 10, 10, -0.1, 10], floor * 3);
		itemIndices.push(floor, floor + 2, floor + 1, floor + 1, floor + 2, floor + 3);
		const full = await native.buildTriangleBvh(itemPositions, new Uint32Array(itemIndices));
		assert.equal(full.triangleCount, items * 24 + 2);
		const reduced = await native.buildTriangleBvh(itemPositions, new Uint32Array(itemIndices), { maxTriangles: 2000 });
		assert.ok(reduced.triangleCount > 2 && reduced.triangleCount <= items * 12 + 2, `simplified to ${reduced.triangleCount}`);
		assert.equal(reduced.triangles.length, reduced.triangleCount * 12);
		// 根的 4 个子节点合起来还是盖住地板和商品
		const root = new Float32Array(reduced.nodes.buffer, reduced.nodes.byteOffset, 32);
		const span = (k, pick) => pick(...[0, 1, 2, 3].map((i) => root[k * 4 + i]).filter((v) => Math.abs(v) < 3e38));
		assert.ok(span(0, Math.min) <= 0 && span(3, Math.max) >= 10 && span(1, Math.min) <= -0.1 && span(4, Math.max) > 1, `still spans the floor and the items: ${[...root.subarray(0, 24)]}`);
		// 没超过上限的不动
		assert.equal((await native.buildTriangleBvh(itemPositions, new Uint32Array(itemIndices), { maxTriangles: 100000 })).triangleCount, items * 24 + 2);
		const boxes = new Float32Array([0, 0, 0, 1, 1, 1, 5, 0, 0, 6, 1, 1, 10, 0, 0, 11, 1, 1]);
		const tlas = await native.buildBoxBvh(boxes);
		assert.deepEqual([...tlas.order].sort(), [0, 1, 2]);
	});

	it('evaluates vector terrain: noise base, flatten polygons and strokes, seamless pages far from the origin', async () => {
		using terrain = native.createTerrain({
			noise: { seed: 7, octaves: 8, scale: 800, height: 150, erosion: 0.3, ridged: 0.3, warp: 200 },
			features: [{ type: 'flatten', points: [[0, 0], [100, 0], [100, 100], [0, 100]], height: 12, falloff: 20 }],
			strokes: [{ op: 'raise', x: 500, z: 500, radius: 50, strength: 30 }],
		});
		// 多边形里压平，过渡带外是原来的噪声
		assert.equal(terrain.height(50, 50), 12);
		assert.notEqual(terrain.height(-200, 50), 12);
		// 一页的点和单点同值，相邻两页的接缝逐位相同
		const page = await terrain.heights(64, 0, 4, 17, 17);
		assert.equal(page.length, 17 * 17);
		assert.equal(page[3 * 17 + 5], terrain.height(64 + 20, 12));
		const right = await terrain.heights(128, 0, 4, 17, 17);
		for (let j = 0; j < 17; j++) assert.equal(page[j * 17 + 16], right[j * 17]);
		// 笔画抬高了中心
		using plain = native.createTerrain({ noise: { seed: 7, octaves: 8, scale: 800, height: 150, erosion: 0.3, ridged: 0.3, warp: 200 } });
		assert.ok(Math.abs(terrain.height(500, 500) - plain.height(500, 500) - 30) < 1e-3);
		// 两万公里外照样连续：隔半米的两个点高度不同又差得不多（单精度的坐标在那里一个刻度 2 米，两点会落成同一个）
		const a = plain.height(20000000, -15000000), b = plain.height(20000000.5, -15000000);
		assert.ok(Number.isFinite(a) && Math.abs(a - b) < 1 && a !== b);
		assert.throws(() => native.createTerrain({ noise: { scale: 0 } }), /scale and lacunarity must be positive/);
		assert.throws(() => native.createTerrain({ noise: {}, features: [{ type: 'flatten', points: [[0, 0], [1, 0]], height: 0 }] }), /at least 3 points/);
	});

	it('carves gullies into the noise base: deterministic, bounded by the strength, under features and strokes', async () => {
		const noise = { seed: 3, octaves: 8, scale: 1200, height: 300, ridged: 0.2, warp: 150 };
		const gullies = { strength: 25, scale: 300, octaves: 4, detail: 1.5, valley: -100, peak: 200 };
		using plain = native.createTerrain({ noise });
		using eroded = native.createTerrain({ noise: { ...noise, gullies }, features: [{ type: 'flatten', points: [[0, 0], [100, 0], [100, 100], [0, 100]], height: 12 }] });
		using again = native.createTerrain({ noise: { ...noise, gullies } });
		// 冲沟只改底：每一点和不带冲沟的差不超过深度，而且确实改了
		let changed = 0, worst = 0;
		for (let i = 0; i < 400; i++) {
			const x = 200 + (i % 20) * 37.3, z = -900 + Math.floor(i / 20) * 41.7;
			const d = Math.abs(again.height(x, z) - plain.height(x, z));
			worst = Math.max(worst, d);
			if (d > 0.5) changed++;
		}
		assert.ok(worst <= gullies.strength + 1e-3, `moved by ${worst} m`);
		assert.ok(changed > 200, `only ${changed} of 400 points changed`);
		// 同样的描述逐位相同；页和单点一致（页的块各算各的也接得上）
		assert.equal(again.height(345.5, -678.25), native.createTerrain({ noise: { ...noise, gullies } }).height(345.5, -678.25));
		const page = await again.heights(1000, 1000, 8, 9, 9);
		assert.equal(page[4 * 9 + 7], again.height(1000 + 56, 1000 + 32));
		// 地物压平照旧压在冲沟上面
		assert.equal(eroded.height(50, 50), 12);
		assert.throws(() => native.createTerrain({ noise: { gullies: { strength: 10, scale: 0 } } }), /gullies need/);
	});

	it('flattens terrain along path features (roads): conform, carve, fill, interpolated heights, long paths', () => {
		const noise = { seed: 5, octaves: 6, scale: 600, height: 80 };
		using plain = native.createTerrain({ noise });
		// 一条从 (0, 10, 0) 爬到 (200, 30, 0) 的路，宽 8 米、往外 10 米过渡
		const road = { type: 'path', points: [[0, 10, 0], [100, 20, 0], [200, 30, 0]], width: 8, falloff: 10 };
		using conform = native.createTerrain({ noise, features: [road] });
		// 路中间、路面边上逐位是路的高度（按线段插值）；过渡带外是原来的地
		assert.equal(conform.height(50, 0), 15);
		assert.equal(conform.height(150, 3.9), 25);
		assert.equal(conform.height(150, 30), plain.height(150, 30));
		const mid = conform.height(150, 9), before = plain.height(150, 9);
		assert.ok(Math.min(before, 25) - 1e-3 <= mid && mid <= Math.max(before, 25) + 1e-3, 'the falloff blends between the ground and the road');
		// 只削：原来低于路的地方不动；只填：原来高于路的地方不动
		using carve = native.createTerrain({ noise, features: [{ ...road, mode: 'carve' }] });
		using fill = native.createTerrain({ noise, features: [{ ...road, mode: 'fill' }] });
		for (let x = 5; x < 200; x += 7) {
			const ground = plain.height(x, 0);
			assert.ok(carve.height(x, 0) <= ground + 1e-4 && fill.height(x, 0) >= ground - 1e-4);
		}
		// 长路（2000 个点、20 公里）：每个点按格子只看附近的线段
		const long = Array.from({ length: 2000 }, (_, i) => [i * 10, 5 + Math.sin(i / 50) * 3, Math.sin(i / 30) * 40]);
		using winding = native.createTerrain({ noise, features: [{ type: 'path', points: long, width: 10, falloff: 5 }] });
		assert.equal(winding.height(long[1234][0], long[1234][2]), Math.fround(long[1234][1]));
		assert.throws(() => native.createTerrain({ noise, features: [{ type: 'path', points: [[0, 0, 0], [1, 0, 0]], width: 0 }] }), /positive width/);
		// 路网：东西向的路在 (100, ·, 0) 是 20 米高，往南岔出一条从 20 米降到 0 的路；岔路的过渡带不能盖过主路的路面
		const branch = { type: 'path', points: [[100, 20, 0], [100, 0, 100]], width: 8, falloff: 30 };
		using network = native.createTerrain({ noise, features: [road, branch] });
		for (const x of [70, 85, 115, 130]) assert.ok(Math.abs(network.height(x, 0) - conform.height(x, 0)) < 1e-3, `main road at x = ${x}: ${network.height(x, 0)} vs ${conform.height(x, 0)}`);
		// 岔路的路面上是岔路的高度
		assert.ok(Math.abs(network.height(100, 50) - 10) < 1e-3, `branch at its middle: ${network.height(100, 50)}`);
	});

	it('collides with a height field (terrain tile) on a static body, releases and reuses shape handles', () => {
		using world = native.createPhysicsWorld(8);
		// 33 × 33 个高度、格距 2 米：沿 x 每米升 0.5 米的斜坡
		const n = 33, spacing = 2;
		const heights = Float32Array.from({ length: n * n }, (_, i) => (i % n) * spacing * 0.5);
		const field = world.createHeightfield(heights, n, spacing);
		world.addBody({ motion: 'static', position: [100, 0, 100], shapes: [{ type: 'heightfield', heightfield: field }] });
		// 本地原点是第一个样本：(100 + 20, ·, 100 + 10) 处地面高 10
		const hit = world.rayCast([120, 50, 110], [0, -1, 0], 100);
		assert.ok(hit && Math.abs(hit.point[1] - 10) < 0.05, `hit at ${hit?.point}`);
		// 球落到斜坡上停住（会往下滚：只看它没穿过去）
		const ball = world.addBody({ motion: 'dynamic', position: [140, 30, 130], shapes: [{ type: 'sphere', radius: 0.5 }] });
		for (let i = 0; i < 120; i++) world.step(1 / 60);
		const y = world.positions()[ball * 3 + 1], x = world.positions()[ball * 3];
		assert.ok(y > (x - 100) * 0.5 - 0.1, `ball at (${x}, ${y}) fell through the slope`);
		assert.throws(() => world.addBody({ motion: 'dynamic', position: [0, 0, 0], shapes: [{ type: 'heightfield', heightfield: field }] }), /only be used on a static body/);
		// 释放后句柄复用；已经用着它的物体照旧碰撞
		world.releaseShape(field);
		assert.throws(() => world.addBody({ motion: 'static', position: [0, 0, 0], shapes: [{ type: 'heightfield', heightfield: field }] }), /does not exist/);
		assert.equal(world.createHeightfield(heights, n, spacing), field);
		assert.ok(world.rayCast([120, 50, 110], [0, -1, 0], 100));
	});

	it('reports the slots whose world matrix changed in the last update', () => {
		using transforms = native.createTransforms(8);
		const [still, mover, child, spare] = transforms.allocate(4);
		const changed = () => [...transforms.changedSlots()].sort((a, b) => a - b);
		transforms.setParent(child, mover);
		transforms.position[mover * 3] = 1;
		transforms.update();
		// 刚分配的都算变了
		assert.deepEqual(changed(), [still, mover, child, spare]);
		transforms.update();
		assert.deepEqual(changed(), []);
		// 动父节点，子节点跟着变；updateSlot 先算过的，下一次 update 照样报（每个只一次）
		transforms.position[mover * 3] = 2;
		transforms.updateSlot(child);
		transforms.update();
		assert.deepEqual(changed(), [mover, child]);
		// 写一样的值不算变
		transforms.position[mover * 3] = 2;
		transforms.update();
		assert.deepEqual(changed(), []);
		// 释放的槽位也报
		transforms.free(spare);
		transforms.update();
		assert.deepEqual(changed(), [spare]);
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
		// 1 × 1 的无损 WebP（stb_image 不认，走 libwebp）
		const webp = await tools.decodeImage(Uint8Array.from(atob('UklGRhoAAABXRUJQVlA4TA0AAAAvAAAAEAcQERGIiP4HAA=='), (c) => c.charCodeAt(0)));
		assert.deepEqual([webp.width, webp.height, webp.pixels.length], [1, 1, 4]);
		await assert.rejects(tools.decodeImage(new TextEncoder().encode('RIFF    WEBPVP8 ')), /not a valid WebP/);
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

	it('ragdolls a skeleton: parts fall onto the ground, the pose is written back into the joint matrices, unsimulated joints follow', () => {
		// 髋（根）往上脊椎、头，往下一条腿；腿不进布娃娃，跟着髋
		const skeleton = native.loadSkeleton(tools.buildSkeleton([
			{ parent: -1, name: 'hips' },
			{ parent: 0, translation: [0, 0.5, 0], name: 'spine' },
			{ parent: 1, translation: [0, 0.4, 0], name: 'head' },
			{ parent: 0, translation: [0, -0.9, 0], name: 'leg' },
		]));
		const parents = Int32Array.from([-1, 0, 1, 0]);
		const instance = skeleton.createInstance();
		using batch = new AnimationBatch(native);
		batch.instance(instance);
		batch.evaluate();
		const modelToWorld = Float64Array.from([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 2, 0, 1]);
		const world = native.createPhysicsWorld(32);
		try {
			world.addBody({ shapes: [{ type: 'box', size: [20, 1, 20] }], position: [0, -0.5, 0] });
			const part = (joint, parent, end) => ({ joint, parent, boneEnd: end, radius: 0.12, mass: 10, swing: 0.6, twist: 0.3 });
			const ragdoll = world.addRagdoll([part(0, -1, [0, 0.5, 0]), part(1, 0, [0, 0.4, 0]), part(2, 1, [0, 0.25, 0])], instance, modelToWorld);
			const legBefore = instance.modelMatrices().slice(48, 64);
			// 推一下，让它倒下而不是直直地落
			world.ragdollImpulse(ragdoll, 2, [20, 0, 0]);
			for (let i = 0; i < 180; i++) world.step(1 / 60);
			const root = world.ragdollRoot(ragdoll);
			assert.ok(root.position[1] > 0 && root.position[1] < 1, `the hips lie on the ground: y ${root.position[1]}`);
			world.ragdollPose(ragdoll, instance, modelToWorld, parents);
			const m = instance.modelMatrices();
			// 头掉下来了（模型空间里比开始的 0.9 低得多），腿跟着髋：腿到髋的相对位姿不变
			assert.ok(m[32 + 13] < 0, `the head fell below where the model was: y ${m[32 + 13]}`);
			const hips = m.slice(0, 16), leg = m.slice(48, 64);
			const legFromHips = [leg[12] - hips[12], leg[13] - hips[13], leg[14] - hips[14]];
			const length = Math.hypot(...legFromHips);
			assert.ok(Math.abs(length - 0.9) < 1e-3, `the leg keeps its offset from the hips: ${length}`);
			assert.notDeepEqual([...leg], [...legBefore]);
			// posed：矩阵已经写好的实例不求值（不会被静止姿势盖掉）
			const written = [...instance.modelMatrices()];
			batch.clear().posed(instance);
			batch.evaluate();
			assert.deepEqual([...instance.modelMatrices()], written);
			assert.throws(() => world.addRagdoll([part(0, 0, [0, 1, 0])], instance, modelToWorld), /only root/);
			world.removeRagdoll(ragdoll);
			assert.throws(() => world.ragdollRoot(ragdoll), /does not exist/);
		} finally {
			world.destroy();
			instance.dispose?.();
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

	it('reports glTF material parameters, texture slots and embedded or external images', async () => {
		const positions = Buffer.from(new Float32Array([0, 0, 0, 1, 0, 0, 0, 1, 0]).buffer).toString('base64');
		const png = Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]);
		const gltf = {
			asset: { version: '2.0' },
			extensionsUsed: ['KHR_texture_transform', 'KHR_materials_emissive_strength', 'EXT_texture_webp'],
			buffers: [{ byteLength: 36, uri: `data:application/octet-stream;base64,${positions}` }],
			bufferViews: [{ buffer: 0, byteOffset: 0, byteLength: 36 }],
			accessors: [{ bufferView: 0, componentType: 5126, count: 3, type: 'VEC3', min: [0, 0, 0], max: [1, 1, 0] }],
			images: [{ uri: `data:image/png;base64,${png.toString('base64')}` }, { uri: 'textures/normal.png' }],
			// 第 2 个贴图只有 WebP 的图（EXT_texture_webp，没有后备的 source）
			textures: [{ source: 0 }, { source: 1 }, { extensions: { EXT_texture_webp: { source: 0 } } }],
			materials: [{
				name: 'Brick',
				pbrMetallicRoughness: { baseColorFactor: [0.5, 0.25, 1, 0.8], metallicFactor: 0.2, roughnessFactor: 0.7, baseColorTexture: { index: 0, extensions: { KHR_texture_transform: { scale: [2, 3], offset: [0.5, 0.25] } } } },
				normalTexture: { index: 1, scale: 0.6 },
				emissiveFactor: [1, 0.5, 0],
				emissiveTexture: { index: 2 },
				extensions: { KHR_materials_emissive_strength: { emissiveStrength: 4 } },
				alphaMode: 'MASK', alphaCutoff: 0.3, doubleSided: true,
			}],
			meshes: [{ primitives: [{ attributes: { POSITION: 0 }, material: 0 }] }],
			nodes: [{ mesh: 0 }],
			scenes: [{ nodes: [0] }],
		};
		const scene = await tools.loadScene(new TextEncoder().encode(JSON.stringify(gltf)), 'gltf');
		try {
			const near = (actual, expected) => actual.forEach((v, i) => assert.ok(Math.abs(v - expected[i]) < 1e-5, `${actual} is not near ${expected}`));
			const [info] = scene.materialInfos;
			near(info.baseColor, [0.5, 0.25, 1, 0.8]);
			near([info.metallic, info.roughness, info.normalScale, info.alphaCutoff], [0.2, 0.7, 0.6, 0.3]);
			near(info.emissive, [4, 2, 0]);
			assert.equal(info.alphaMode, 'mask');
			assert.equal(info.doubleSided, true);
			assert.deepEqual(info.baseColorTexture, { image: 0, scale: [2, 3], offset: [0.5, 0.25] });
			assert.equal(info.normalTexture.image, 1);
			assert.equal(info.emissiveTexture.image, 0);
			assert.equal(scene.imageCount, 2);
			const embedded = scene.image(0);
			assert.deepEqual([...embedded.data], [...png]);
			assert.equal(embedded.mime, 'image/png');
			assert.deepEqual(scene.image(1), { data: null, uri: 'textures/normal.png', mime: '' });
		} finally {
			scene.destroy();
		}
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

	it('fractures a closed mesh into Voronoi pieces that fill its volume, marking the cut faces', async () => {
		const at = (x, y, z) => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1];
		const box = await tools.brushCsg([{ kind: 'box', operation: 'add', transform: at(0, 0, 0), size: [2, 1, 1] }]);
		const result = await tools.fractureMesh(box, 8, 7);
		assert.ok(result.pieceCount >= 6 && result.pieceCount <= 8, `pieces: ${result.pieceCount}`);
		// 体积守恒：各块的体积加起来是 2 立方米（按三角形的有符号体积算）
		let volume = 0;
		const p = result.positions, idx = result.indices;
		for (let t = 0; t < idx.length; t += 3) {
			const [a, b, c] = [idx[t] * 3, idx[t + 1] * 3, idx[t + 2] * 3];
			volume += (p[a] * (p[b + 1] * p[c + 2] - p[b + 2] * p[c + 1]) - p[a + 1] * (p[b] * p[c + 2] - p[b + 2] * p[c]) + p[a + 2] * (p[b] * p[c + 1] - p[b + 1] * p[c])) / 6;
		}
		assert.ok(Math.abs(volume - 2) < 1e-3, `volume ${volume}`);
		const interior = result.interior.reduce((sum, flag) => sum + flag, 0);
		assert.ok(interior > 0 && interior < result.interior.length, `cut faces: ${interior} of ${result.interior.length}`);
		// 同一个种子每次一样
		const again = await tools.fractureMesh(box, 8, 7);
		assert.deepEqual([...again.pieceOfTriangle], [...result.pieceOfTriangle]);
	});

	it('combines greybox brushes (Manifold) and reports whose surface each triangle is', async () => {
		const at = (x, y, z) => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1];
		const solid = await tools.brushCsg([
			{ kind: 'box', operation: 'add', transform: at(0, 0, 0), size: [4, 1, 4] },
			{ kind: 'prism', operation: 'add', transform: at(-2, 0.5, -2), points: [[0, 0], [2, 0], [2, 1], [1, 1], [1, 2], [0, 2]], height: 2 },
			{ kind: 'cylinder', operation: 'subtract', transform: at(1, 0, 1), height: 2, bottomRadius: 0.5, topRadius: 0.5, segments: 24 },
		]);
		assert.equal(solid.indices.length % 3, 0);
		assert.equal(solid.brushOfTriangle.length, solid.indices.length / 3);
		assert.deepEqual(new Set(solid.brushOfTriangle), new Set([0, 1, 2]));
		// 圆柱挖穿了板子：板子在圆柱里的那块没了，留下的是圆柱的内壁
		const y = [];
		for (let v = 0; v < solid.positions.length; v += 3) y.push(solid.positions[v + 1]);
		assert.ok(Math.max(...y) > 2.4 && Math.min(...y) > -0.6);
		await assert.rejects(tools.brushCsg([{ kind: 'prism', operation: 'add', transform: at(0, 0, 0), points: [[0, 0], [1, 1]], height: 1 }]), /prism needs at least 3 points/);
	});

	it('meshes smoothly blended SDF shapes (Manifold level set) with smooth normals', async () => {
		const at = (x, y, z) => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1];
		const solid = await tools.sdfMesh([
			{ kind: 'ellipsoid', operation: 'add', smoothness: 0, transform: at(0, 0, 0), radii: [0.6, 0.8, 0.6] },
			{ kind: 'sphere', operation: 'add', smoothness: 0.3, transform: at(0, 1, 0), radius: 0.4 },
			{ kind: 'capsule', operation: 'subtract', smoothness: 0.05, transform: at(0, 1.1, 0.4), halfHeight: 0.05, radius: 0.1 },
		], 0.05);
		assert.equal(solid.indices.length % 3, 0);
		assert.equal(solid.normals.length, solid.positions.length);
		assert.equal(solid.shapeOfTriangle.length, solid.indices.length / 3);
		assert.deepEqual(new Set(solid.shapeOfTriangle), new Set([0, 1, 2]));
		// 法线是单位长度
		for (let v = 0; v < solid.normals.length; v += 3) assert.ok(Math.abs(Math.hypot(solid.normals[v], solid.normals[v + 1], solid.normals[v + 2]) - 1) < 1e-3);
		await assert.rejects(tools.sdfMesh([{ kind: 'sphere', operation: 'subtract', smoothness: 0, transform: at(0, 0, 0), radius: 1 }], 0.05), /no shape adds anything/);
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
		// Arial 要求 ppem 取整（head flags 第 3 位）；名字在 19 个整数之后（读错位置时会少掉前几个字母）
		if (fontPath.endsWith('arial.ttf')) assert.equal(description.headFlags & 8, 8);
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
