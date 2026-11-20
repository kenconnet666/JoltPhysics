// 线程数的基准：不同的工作线程数下，运行时模块的启动、物理步进、并行任务（导航网格分块构建）各要多久，那段时间里平均几个核在忙。
// 测的是 JS/dist 里打包出去的正式版（先跑 Scripts/build-wasm.ps1）。每种线程数在单独的子进程里跑，线程池互不影响；
// 每种跑 RUNS 次取中位数。测之前看一眼 CPU 负载：别的进程占着核时数字不可信。
//   node Tests/Bench/threads.mjs                 # 默认 1 2 4 8 12 16 24 和（逻辑核数 - 1）
//   node Tests/Bench/threads.mjs 4 8 35
import { execFileSync } from 'node:child_process';
import { availableParallelism } from 'node:os';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const RUNS = 3;
const self = fileURLToPath(import.meta.url);
const dist = path.join(path.dirname(self), '..', '..', 'JS', 'dist');

/** 起伏的地面网格（导航网格要有坡和坑才有活干）。 */
function terrain(size, cells) {
	const vertices = [], triangles = [];
	for (let z = 0; z <= cells; ++z)
		for (let x = 0; x <= cells; ++x) {
			const px = -size / 2 + (size * x) / cells, pz = -size / 2 + (size * z) / cells;
			vertices.push(px, Math.sin(px * 0.15) * Math.cos(pz * 0.11) * 3, pz);
		}
	for (let z = 0; z < cells; ++z)
		for (let x = 0; x < cells; ++x) {
			const i = z * (cells + 1) + x;
			triangles.push(i, i + cells + 1, i + 1, i + 1, i + cells + 1, i + cells + 2);
		}
	return { vertices: new Float32Array(vertices), triangles: new Int32Array(triangles) };
}

/** 跑 inWork，返回墙钟毫秒和这段时间里平均有几个核在忙（进程所有线程的 CPU 时间 / 墙钟时间；比加速比大很多就是有线程在空转）。 */
async function measure(inWork) {
	const cpu = process.cpuUsage(), start = performance.now();
	await inWork();
	const ms = performance.now() - start, used = process.cpuUsage(cpu);
	return { ms, cores: (used.user + used.system) / 1000 / ms };
}

/** 物理：一个世界里放好箱子，热身后连续步进 STEPS 步（不让它们睡着），返回每步的毫秒和忙着的核。 */
async function physicsBench(native, place, count) {
	const STEPS = 240;
	const world = native.createPhysicsWorld(count + 1);
	world.addBody({ shapes: [{ type: 'box', size: [400, 1, 400] }], position: [0, -0.5, 0] });
	for (let i = 0; i < count; ++i)
		world.addBody({ motion: 'dynamic', shapes: [{ type: 'box', size: [1, 1, 1] }], position: place(i), neverSleep: true });
	for (let i = 0; i < 10; ++i) world.step(1 / 60);
	const r = await measure(() => { for (let i = 0; i < STEPS; ++i) world.step(1 / 60); });
	world.destroy?.();
	return { ms: r.ms / STEPS, cores: r.cores };
}

async function runOne(workerThreads) {
	const { createStrideNative } = await import(pathToFileURL(path.join(dist, 'stride-native.js')));
	const features = await Promise.all(['physics', 'texture', 'navigation'].map(async (name) => (await import(pathToFileURL(path.join(dist, `${name}.js`))))[name]));

	const start = performance.now();
	const native = await createStrideNative({ workerThreads, features });
	const startupMs = performance.now() - start;

	// 一大堆：4000 个箱子从一个方阵里落下，堆成一个大岛（求解的并行度最差的情况）
	const SIDE = Math.ceil(Math.cbrt(4000));
	const pile = await physicsBench(native, (i) => [((i % SIDE) - SIDE / 2) * 1.1, 1 + Math.floor(i / (SIDE * SIDE)) * 1.1, ((Math.floor(i / SIDE) % SIDE) - SIDE / 2) * 1.1], 4000);
	// 很多小堆：100 堆、每堆 40 个，堆和堆隔开（开放世界里互不接触的物体群）
	const piles = await physicsBench(native, (i) => {
		const group = Math.floor(i / 40), k = i % 40;
		return [((group % 10) - 5) * 30 + (k % 4) * 1.1, 1 + Math.floor(k / 16) * 1.1, (Math.floor(group / 10) - 5) * 30 + (Math.floor(k / 4) % 4) * 1.1];
	}, 4000);

	// 并行任务：导航网格分块构建。buildNavMesh 同步返回之前做完了检查输入和三角形分块（单线程），之后各块并行建，最后单线程拼起来。
	// 建三次：第一次里 V8 还在把 wasm 的热函数换成优化过的代码（动态分层编译，线程越多越慢），后两次才是并行本身的效率
	const { vertices, triangles } = terrain(300, 300);
	const builds = [];
	let prepareMs = 0;
	for (let i = 0; i < 3; ++i) {
		let navMesh;
		builds.push(await measure(async () => {
			const begin = performance.now();
			const pending = native.buildNavMesh(vertices, triangles, { tileSize: 32 });
			prepareMs = performance.now() - begin;
			navMesh = await pending;
		}));
		navMesh.destroy?.();
	}
	const [cold, ...warm] = builds;
	const nav = { coldMs: cold.ms, ms: (warm[0].ms + warm[1].ms) / 2, cores: (warm[0].cores + warm[1].cores) / 2 };

	// 空闲：任务系统没活干时，工作线程应该睡着，不占 CPU
	const idle = await measure(() => new Promise((resolve) => setTimeout(resolve, 500)));

	native.shutdown();
	return { workerThreads, startupMs, pileMs: pile.ms, pileCores: pile.cores, pilesMs: piles.ms, pilesCores: piles.cores, navColdMs: nav.coldMs, navMs: nav.ms, navCores: nav.cores, prepareMs, idleCores: idle.cores };
}

const single = process.argv.indexOf('--one');
if (single >= 0) {
	console.log(JSON.stringify(await runOne(Number(process.argv[single + 1]))));
	process.exit(0);
}

const median = (values) => [...values].sort((a, b) => a - b)[Math.floor(values.length / 2)];
const counts = process.argv.slice(2).map(Number).filter((n) => n > 0);
const list = counts.length > 0 ? counts : [...new Set([1, 2, 4, 8, 12, 16, 24, availableParallelism() - 1])].sort((a, b) => a - b);
console.log(`logical cores: ${availableParallelism()}, median of ${RUNS} runs; cores = average busy cores in that phase`);
console.log('threads  startup  one pile ms/step cores  100 piles ms/step cores  navmesh first ms  warm ms cores (prepare ms)  idle cores');
for (const n of list) {
	const runs = [];
	for (let i = 0; i < RUNS; ++i) runs.push(JSON.parse(execFileSync(process.execPath, [self, '--one', String(n)], { encoding: 'utf8' }).trim().split('\n').at(-1)));
	const m = Object.fromEntries(Object.keys(runs[0]).map((key) => [key, median(runs.map((r) => r[key]))]));
	const f = (value, digits, width) => value.toFixed(digits).padStart(width);
	console.log(`${f(n, 0, 7)}  ${f(m.startupMs, 0, 7)}  ${f(m.pileMs, 2, 16)} ${f(m.pileCores, 1, 5)}  ${f(m.pilesMs, 2, 17)} ${f(m.pilesCores, 1, 5)}  ${f(m.navColdMs, 0, 17)}  ${f(m.navMs, 0, 7)} ${f(m.navCores, 1, 5)} ${f(m.prepareMs, 0, 11)}  ${f(m.idleCores, 2, 10)}`);
}
