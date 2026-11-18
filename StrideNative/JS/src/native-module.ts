// StrideNative 两个 wasm 模块（stride-native、stride-tools）共用的 JS 侧：实例化、内存视图、结构体读写、
// 错误和原生任务的等待。两个模块都是多线程的，内存是 SharedArrayBuffer。
// 这是 npm 包 @stride-ts/native 的源码：tsc 编译进 dist/（相对导入写 .ts，产出里改成 .js），
// 和 Emscripten 产出的 .mjs、.wasm 放在一起，用 pnpm pack 打包。

/** C 侧 SN_ERROR_* 对应的错误种类。 */
export type NativeErrorCode = 'invalid-argument' | 'invalid-handle' | 'failed' | 'not-initialized';

const ERROR_CODES: Readonly<Record<number, NativeErrorCode>> = {
	[-1]: 'invalid-argument',
	[-2]: 'invalid-handle',
	[-3]: 'failed',
	[-4]: 'not-initialized',
};

/** 原生调用失败：code 是错误种类，status 是 C 侧返回的负数。 */
export class NativeError extends Error {
	readonly code: NativeErrorCode;
	readonly status: number;

	constructor(message: string, status: number) {
		super(message);
		this.name = 'NativeError';
		this.status = status;
		this.code = ERROR_CODES[status] ?? 'failed';
	}
}

import type { CoreExport, NativeExports } from './native-exports.ts';

export type { CoreExport };

export interface NativeDebugSnapshot {
	readonly counters: Readonly<Record<string, number>>;
	/** 每项：调用次数、总毫秒数、最长一次。 */
	readonly timings: Readonly<Record<string, { readonly calls: number; readonly ms: number; readonly maxMs: number }>>;
	/** wasm 里有：堆大小（heapBytes）和 malloc 分出去的字节（allocatedBytes）。 */
	readonly memory: Readonly<Record<string, number>>;
}

export interface NativeLogLine {
	readonly level: 'debug' | 'info' | 'warn' | 'error';
	readonly message: string;
}

/** SN_LOG_DEBUG..SN_LOG_ERROR。 */
const LOG_LEVELS = ['debug', 'info', 'warn', 'error'] as const;

/** C 函数在 JS 里的样子：指针、句柄、整数和 float 都是 number（int64 参数会是 bigint，目前没有）。 */
export type NativeFunction = (...args: number[]) => number;

/**
 * Emscripten 编出的模块：C 函数以 `_` 加名字导出，外加我们要用的运行时成员。签名（参数名、个数、注释）从头文件生成
 * （native-exports.ts），这个模块有哪些按名字挑出来：写错名字、传错参数个数在编译期就报，IDE 能跳到声明。
 */
export type CoreModule<E extends string = never> = {
	readonly wasmMemory: WebAssembly.Memory;
	UTF8ToString(ptr: number): string;
} & Pick<NativeExports, Extract<`_${CoreExport | E}`, keyof NativeExports>>;

/** Emscripten 的模块工厂（MODULARIZE + EXPORT_ES6 生成的默认导出）。 */
export type CoreFactory<E extends string> = (options: Record<string, unknown>) => Promise<CoreModule<E>>;

export interface ModuleOptions {
	/** 任务系统的工作线程数；默认 navigator.hardwareConcurrency - 1（至少 1）。 */
	workerThreads?: number;
	/**
	 * 用调试版的运行时模块（stride-native-core-debug）：打开 Jolt 的断言（失败时 console.error 带 C++ 调用栈）和
	 * Emscripten 的运行时检查（ASSERTIONS、栈溢出检查）。比正式版慢，开发时用。工具模块没有调试版，忽略这一项。
	 */
	debug?: boolean;
	/** Emscripten 的 locateFile，wasm 放在别的路径时用。 */
	locateFile?: (path: string, prefix: string) => string;
	/**
	 * 运行时要加载的侧模块：内置的功能从各自的入口导入（`import { physics } from '@stride-ts/native/physics'`，
	 * 还有 texture、navigation），第三方插件给出自己的 NativeFeature。没导入的功能，打包器不会输出它的 wasm。
	 * 侧模块只能在这里加载（每个线程都要加载一份），之后不能再加。工具模块不用这一项。
	 */
	features?: readonly NativeFeature[];
}

/** NativeFeature 的幻影类型参数的键，只在类型里出现。 */
export declare const FEATURE_EXPORTS: unique symbol;

/**
 * 一个侧模块：内置功能（物理、贴图、导航）或第三方插件。E 是它导出的 C 函数名，NativeModule.plugin 按它给出
 * 带类型的函数表；内置功能的函数已经在 StrideNative 的方法里包好，不用写。
 */
export interface NativeFeature<E extends string = string> {
	/** 和 C 侧 SN_PLUGIN(name) 的名字一致（C 标识符）；加载时调用 sn_plugin_abi_<name> 核对插件 ABI 版本。 */
	readonly name: string;
	readonly url: URL;
	/**
	 * 调试版运行时（debug: true）用的文件。内置功能都有，它们和调试版的核心一起编译（Jolt 断言会改变结构体布局）；
	 * 只用插件 C API 的第三方插件不需要，没有时用 url。
	 */
	readonly debugUrl?: URL;
	readonly [FEATURE_EXPORTS]?: E;
}

/** 实例化好的模块和它加载的侧模块（功能名到导出函数）。 */
export interface LoadedModule<E extends string> {
	readonly module: CoreModule<E>;
	readonly features: ReadonlyMap<string, Readonly<Record<string, NativeFunction>>>;
}

const FEATURE_NAME = /^[A-Za-z_]\w*$/;

const TASK_RUNNING = 0;
const TASK_DONE = 1;

const isNode = typeof process !== 'undefined' && process.versions?.node != null;

/** 默认开的工作线程数：每个核一个，减去当前线程。 */
export function defaultWorkerThreads(): number {
	const cores = globalThis.navigator?.hardwareConcurrency ?? 4;
	return Math.max(1, cores - 1);
}

/** 模块自己的 wasm：Emscripten 请求它时用的文件名，和打包器改写过的地址。 */
export interface ModuleWasm {
	readonly file: string;
	readonly url: URL;
}

/**
 * 实例化 Emscripten 模块并启动任务系统。浏览器里只有跨源隔离的页面（COOP + COEP 响应头）才能用共享内存。
 * 要加载侧模块时必须给出 wasm：给了 locateFile，Emscripten 就不再用它自己的 new URL(..., import.meta.url)，
 * 打包器给模块 wasm 改的（带哈希的）地址只能由调用方的 new URL 提供。
 */
export async function instantiate<E extends string>(factory: CoreFactory<E>, { workerThreads = defaultWorkerThreads(), debug = false, locateFile, features = [] }: ModuleOptions = {}, wasm?: ModuleWasm): Promise<LoadedModule<E>> {
	if (!isNode && !globalThis.crossOriginIsolated) {
		throw new NativeError('StrideNative needs a cross-origin isolated page: serve it with ' +
			'"Cross-Origin-Opener-Policy: same-origin" and "Cross-Origin-Embedder-Policy: require-corp"', -4);
	}
	// 侧模块在启动时加载（主线程和线程池里的每个线程都加载，之后才运行任何代码）。Emscripten 按 <名字>.wasm
	// 登记它们，实际地址由 locateFile 给出
	const libraries = new Map<string, URL>();
	if (features.length > 0 && !wasm) throw new NativeError('this module cannot load features', -3);
	for (const feature of features) {
		if (!FEATURE_NAME.test(feature.name)) throw new TypeError(`feature name must be a C identifier, not '${feature.name}'`);
		if (libraries.has(`${feature.name}.wasm`)) throw new TypeError(`feature '${feature.name}' is listed twice`);
		libraries.set(`${feature.name}.wasm`, debug ? feature.debugUrl ?? feature.url : feature.url);
	}
	const locate = (path: string, prefix: string) => {
		const url = libraries.get(path) ?? (path === wasm?.file && !locateFile ? wasm.url : undefined);
		if (url) return isNode && url.protocol === 'file:' ? decodeURIComponent(url.pathname.replace(/^\/(\w:)/, '$1')) : url.href;
		return locateFile ? locateFile(path, prefix) : prefix + path;
	};
	let module: CoreModule<E>;
	try {
		// Worker 池必须装得下任务系统的所有线程：Jolt 是同步创建它们的
		module = await factory({
			workerPoolSize: workerThreads,
			...(features.length > 0 ? { dynamicLibraries: [...libraries.keys()], locateFile: locate } : locateFile ? { locateFile } : {}),
		});
	} catch (error) {
		// 插件用了运行时没导出的符号时，Emscripten 在这里报 undefined symbol
		const loading = features.length > 0 ? ` with features ${features.map((f) => f.name).join(', ')}` : '';
		throw new NativeError(`loading StrideNative${loading} failed: ${error instanceof Error ? error.message : String(error)}`, -3);
	}
	const loaded = new Map<string, Record<string, NativeFunction>>();
	const ldso = (module as unknown as { LDSO?: { loadedLibsByName: Record<string, { exports: Record<string, unknown> } | undefined> } }).LDSO;
	const abiVersion = (module as unknown as Partial<Record<'_sn_plugin_abi_version', NativeFunction>>)._sn_plugin_abi_version;
	for (const { name } of features) {
		const library = ldso?.loadedLibsByName[`${name}.wasm`];
		if (!library || !abiVersion) throw new NativeError(`feature '${name}' was not loaded: only the runtime module (stride-native-core) loads features`, -3);
		const exports: Record<string, NativeFunction> = {};
		for (const [symbol, value] of Object.entries(library.exports)) if (typeof value === 'function') exports[symbol] = value as NativeFunction;
		const pluginAbi = exports[`sn_plugin_abi_${name}`];
		if (!pluginAbi) throw new NativeError(`feature '${name}' does not export sn_plugin_abi_${name}: declare it with SN_PLUGIN(${name})`, -3);
		if (pluginAbi() !== abiVersion()) {
			throw new NativeError(`feature '${name}' was built for plugin ABI ${pluginAbi()}, the runtime has ${abiVersion()}`, -3);
		}
		// 侧模块的导出只并进了 Emscripten 的全局符号表：挂到模块上，内置功能的绑定照常用 module._sn_xxx 调用
		for (const [symbol, fn] of Object.entries(exports)) {
			if (!(`_${symbol}` in module)) (module as unknown as Record<string, unknown>)[`_${symbol}`] = fn;
		}
		loaded.set(name, exports);
	}
	const status = module._sn_init(workerThreads);
	if (status !== 0) {
		throw new NativeError(`sn_init failed: ${module.UTF8ToString(module._sn_last_error())}`, status);
	}
	return { module, features: loaded };
}

/** 原生任务结束后读结果用的东西。 */
export interface TaskResult {
	result(index: number): number;
	outputCount(): number;
	output<A extends TypedArray>(type: TypedArrayType<A>, index: number): A;
}

export type TypedArray = Uint8Array | Int32Array | Uint32Array | Float32Array | Float64Array | BigInt64Array | BigUint64Array;
export interface TypedArrayType<A extends TypedArray> {
	new (buffer: ArrayBufferLike, byteOffset: number, length: number): A;
	readonly BYTES_PER_ELEMENT: number;
}

/** 模块包装类的基类；E 是这个模块自己导出的 C 函数名。 */
export class NativeModule<E extends string = never> implements Disposable {
	/** @internal Emscripten 模块，派生类和同一层的包装类用。 */
	readonly module: CoreModule<E>;
	readonly #features: LoadedModule<E>['features'];
	#shutDown = false;

	constructor({ module, features }: LoadedModule<E>) {
		this.module = module;
		this.#features = features;
	}

	/** 加载了这个功能或插件（createStrideNative 的 features 里有它）。 */
	hasFeature(feature: NativeFeature | string): boolean {
		return this.#features.has(typeof feature === 'string' ? feature : feature.name);
	}

	/**
	 * 插件导出的 C 函数，按 C 的名字调用：`native.plugin(example).sn_example_wave(...)`。指针参数用 alloc、withArrays
	 * 这些方法准备，异步任务用 awaitTask 等。插件没加载时抛 NativeError。
	 */
	plugin<P extends string>(feature: NativeFeature<P>): { readonly [K in P]: NativeFunction } {
		const exports = this.#features.get(feature.name);
		if (!exports) throw new NativeError(`plugin '${feature.name}' is not loaded: pass it to createStrideNative({ features: [...] })`, -4);
		return exports as { readonly [K in P]: NativeFunction };
	}

	/**
	 * 运行时对象（变换存储、物理世界……）的原生句柄，插件的绑定把它传给插件的 C 函数，比如 sn_transforms_columns
	 * 要的变换存储句柄。对象必须属于这个模块，并且没有销毁。
	 */
	handleOf(object: NativeObject<NativeModule<string>>): number {
		if (!object.belongsTo(this)) throw new NativeError(`${object.constructor.name} belongs to another StrideNative module`, -2);
		return object[HANDLE];
	}

	/** 内置功能的方法先调用它：功能没加载时说明怎么加载。 */
	protected requireFeature(name: string, what: string): void {
		if (this.#features.has(name)) return;
		throw new NativeError(`${what} needs the ${name} feature: import { ${name} } from '@stride-ts/native/${name}' and pass it to createStrideNative({ features: [${name}] })`, -4);
	}

	/** 任务系统的工作线程数（等待任务时当前线程也会帮着跑）。 */
	get workerThreadCount(): number {
		return this.module._sn_worker_thread_count();
	}

	/**
	 * 收集调试数据（Include/StrideNative/Debug.h 的计数器、计时和日志）。默认关，关着时埋点几乎没有开销；
	 * stride-ts 的调试桥连上调试 CLI 时打开。
	 */
	get debugCollecting(): boolean {
		return this.module._sn_debug_enabled() !== 0;
	}

	set debugCollecting(on: boolean) {
		this.module._sn_debug_set_enabled(on ? 1 : 0);
	}

	/** 计数器的当前值、各项计时（打开收集以来或上次 resetDebugTimings 以来）和内存。 */
	debugSnapshot(): NativeDebugSnapshot {
		return JSON.parse(this.module.UTF8ToString(this.module._sn_debug_snapshot())) as NativeDebugSnapshot;
	}

	resetDebugTimings(): void {
		this.module._sn_debug_reset_timings();
	}

	/** 取走排队的原生日志（Jolt 的 Trace、断言、物理步进出错……）。收集关着时只有错误会排队。 */
	drainDebugLog(): NativeLogLine[] {
		return (JSON.parse(this.module.UTF8ToString(this.module._sn_debug_drain_log())) as { level: number; message: string }[])
			.map(({ level, message }) => ({ level: LOG_LEVELS[level] ?? 'info', message }));
	}

	/** 停掉任务系统、释放所有原生对象；之后模块不能再用。 */
	shutdown(): void {
		if (this.#shutDown) return;
		this.#shutDown = true;
		this.module._sn_shutdown();
	}

	[Symbol.dispose](): void {
		this.shutdown();
	}

	/** 模块的共享内存。内存增长后 memory.buffer 是新的、更大的对象，所以视图每次现取。 */
	get buffer(): SharedArrayBuffer {
		return this.module.wasmMemory.buffer as unknown as SharedArrayBuffer;
	}

	u8(ptr: number, length: number): Uint8Array { return new Uint8Array(this.buffer, ptr, length); }
	i32(ptr: number, length: number): Int32Array { return new Int32Array(this.buffer, ptr, length); }
	u32(ptr: number, length: number): Uint32Array { return new Uint32Array(this.buffer, ptr, length); }
	f32(ptr: number, length: number): Float32Array { return new Float32Array(this.buffer, ptr, length); }
	f64(ptr: number, length: number): Float64Array { return new Float64Array(this.buffer, ptr, length); }
	/** int64 / uint64 视图：stride-ts 的 i64、decimal、time、datetime、instant 列和 C 的 Values.h 用这个布局。 */
	i64(ptr: number, length: number): BigInt64Array { return new BigInt64Array(this.buffer, ptr, length); }
	u64(ptr: number, length: number): BigUint64Array { return new BigUint64Array(this.buffer, ptr, length); }
	view(ptr: number, length: number): DataView { return new DataView(this.buffer, ptr, length); }

	/** 在模块内存里分配 16 字节对齐的一块，返回偏移；用 free 释放。 */
	alloc(size: number): number {
		const ptr = this.module._sn_alloc(size);
		if (size > 0 && ptr === 0) throw new NativeError(`allocating ${size} bytes failed`, -3);
		return ptr;
	}

	free(ptr: number): void {
		this.module._sn_free(ptr);
	}

	/** 把一个 TypedArray 拷进新分配的原生内存，返回指针。 */
	copyIn(array: ArrayBufferView): number {
		const bytes = new Uint8Array(array.buffer, array.byteOffset, array.byteLength);
		const ptr = this.alloc(bytes.byteLength);
		this.u8(ptr, bytes.byteLength).set(bytes);
		return ptr;
	}

	/** 把原生内存拷成一个普通（非共享）的 TypedArray。 */
	copyOut<A extends TypedArray>(type: TypedArrayType<A>, ptr: number, length: number): A {
		return (new type(this.buffer, ptr, length) as unknown as { slice(): A }).slice();
	}

	/** 把这些数组拷进原生内存、用它们的指针调用 fn，结束后释放；数组可以是 null（指针为 0）。 */
	withArrays<R>(arrays: readonly (ArrayBufferView | null | undefined)[], fn: (...pointers: number[]) => R): R {
		const pointers = arrays.map((array) => (array ? this.copyIn(array) : 0));
		try {
			return fn(...pointers);
		} finally {
			for (const ptr of pointers) if (ptr) this.free(ptr);
		}
	}

	/** withArrays 的异步版：数组一直留到返回的 Promise 结束。 */
	async withArraysAsync<R>(arrays: readonly (ArrayBufferView | null | undefined)[], fn: (...pointers: number[]) => Promise<R>): Promise<R> {
		const pointers = arrays.map((array) => (array ? this.copyIn(array) : 0));
		try {
			return await fn(...pointers);
		} finally {
			for (const ptr of pointers) if (ptr) this.free(ptr);
		}
	}

	/** 分配一块临时内存交给 fn，结束后释放。 */
	withScratch<R>(size: number, fn: (ptr: number) => R): R {
		const ptr = this.alloc(size);
		try {
			return fn(ptr);
		} finally {
			this.free(ptr);
		}
	}

	lastError(): string {
		return this.module.UTF8ToString(this.module._sn_last_error());
	}

	/** 原生调用返回负数（失败）时抛 NativeError，否则原样返回。 */
	check(value: number, what: string): number {
		if (value < 0) throw new NativeError(`${what}: ${this.lastError()}`, value);
		return value;
	}

	/**
	 * 不阻塞线程地等一个原生任务结束，用 read(task) 读结果，然后释放任务。在任务的状态格上用
	 * Atomics.waitAsync 等（模块完成时会 notify）；没有 waitAsync 的环境退回轮询。
	 */
	async awaitTask<R>(task: number, what: string, read: (task: TaskResult) => R = () => undefined as R): Promise<R> {
		this.check(task, what);
		const m = this.module;
		// Node 里正在等的 Atomics.waitAsync 和 Emscripten 的（unref 过的）Worker 都不会让进程活着，
		// 进程可能在任务中途退出；用一个定时器在等待期间撑住事件循环。
		const keepAlive = isNode ? setInterval(() => {}, 1 << 30) : null;
		try {
			const status = this.i32(m._sn_task_status_address(task), 1);
			while (Atomics.load(status, 0) === TASK_RUNNING) {
				if (typeof Atomics.waitAsync === 'function') {
					const wait = Atomics.waitAsync(status, 0, TASK_RUNNING);
					if (wait.async) await wait.value;
				} else {
					await new Promise((resolve) => setTimeout(resolve, 1));
				}
			}
			if (Atomics.load(status, 0) !== TASK_DONE) {
				throw new NativeError(`${what}: ${m.UTF8ToString(m._sn_task_error(task))}`, -3);
			}
			return read({
				result: (index) => m._sn_task_result(task, index),
				outputCount: () => m._sn_task_output_count(task),
				output: (type, index) => {
					const size = m._sn_task_output_size(task, index);
					return this.copyOut(type, m._sn_task_output_data(task, index), size / type.BYTES_PER_ELEMENT);
				},
			});
		} finally {
			if (keepAlive) clearInterval(keepAlive);
			m._sn_task_release(task);
		}
	}

	/** 读原生调用通过一对输出指针分配出来的缓冲，然后释放它。 */
	readOwnedBuffer(call: (dataOut: number, sizeOut: number) => number, what: string): Uint8Array {
		return this.withScratch(8, (out) => {
			this.check(call(out, out + 4), what);
			const [dataPtr, size] = this.u32(out, 2);
			const bytes = this.copyOut(Uint8Array, dataPtr!, size!);
			this.free(dataPtr!);
			return bytes;
		});
	}
}

/** Recast 的设置；没给的字段用 Recast 演示里给人形代理的默认值。 */
export interface NavConfig {
	cellSize?: number;
	cellHeight?: number;
	agentHeight?: number;
	agentRadius?: number;
	agentMaxClimb?: number;
	agentMaxSlopeDegrees?: number;
	/** 以格为单位；各个 tile 并行构建。 */
	tileSize?: number;
	maxEdgeLength?: number;
	maxSimplificationError?: number;
	minRegionSize?: number;
	mergeRegionSize?: number;
	detailSampleDistance?: number;
	detailSampleMaxError?: number;
}

const NAV_FIELDS: readonly [keyof NavConfig, 'f' | 'i'][] = [
	['cellSize', 'f'], ['cellHeight', 'f'], ['agentHeight', 'f'], ['agentRadius', 'f'],
	['agentMaxClimb', 'f'], ['agentMaxSlopeDegrees', 'f'], ['tileSize', 'i'], ['maxEdgeLength', 'f'],
	['maxSimplificationError', 'f'], ['minRegionSize', 'f'], ['mergeRegionSize', 'f'],
	['detailSampleDistance', 'f'], ['detailSampleMaxError', 'f'],
];

export const NAV_CONFIG_SIZE = NAV_FIELDS.length * 4;

/** 写一个 sn_nav_config：先填原生的默认值，再写给了的字段。公开接口在这里检查一次取值。 */
export function writeNavConfig(native: NativeModule<'sn_nav_default_config'>, ptr: number, config: NavConfig = {}): void {
	native.module._sn_nav_default_config(ptr);
	const view = native.view(ptr, NAV_CONFIG_SIZE);
	NAV_FIELDS.forEach(([name, type], index) => {
		const value = config[name];
		if (value === undefined) return;
		if (!Number.isFinite(value) || value < 0 || (type === 'i' && !Number.isInteger(value))) {
			throw new RangeError(`NavConfig.${name} must be a non-negative ${type === 'i' ? 'integer' : 'number'}, got ${value}`);
		}
		if (type === 'i') view.setInt32(index * 4, value, true);
		else view.setFloat32(index * 4, value, true);
	});
}

/** 只在编译期区分的整数：BodyId、AgentId 不能混用。 */
declare const brand: unique symbol;
export type Branded<T, B extends string> = T & { readonly [brand]: B };

/** 公开接口用字符串联合，C 侧用整数；JS 调用方可能传错，所以这里检查。 */
export function enumCode<K extends string>(codes: Readonly<Record<K, number>>, value: K, what: string): number {
	const code = Object.hasOwn(codes, value) ? codes[value] : undefined;
	if (code === undefined) {
		throw new TypeError(`${what} must be one of ${Object.keys(codes).map((k) => `'${k}'`).join(', ')}, not '${String(value)}'`);
	}
	return code;
}

/** 原生句柄的键。不从公开入口导出，用户代码拿不到句柄，只有同一层的包装类互相传。 */
export const HANDLE: unique symbol = Symbol('stride-native handle');

/** 一个原生对象（世界、导航网格、骨架……）的包装：destroy 或 using 释放，释放后再用会抛 invalid-handle。 */
export abstract class NativeObject<M extends NativeModule<string>> implements Disposable {
	protected readonly native: M;
	readonly #handle: number;
	#alive = true;

	/** @internal 由模块的工厂方法创建（createPhysicsWorld、loadNavMesh……）。 */
	constructor(native: M, handle: number) {
		this.native = native;
		this.#handle = handle;
	}

	/** @internal */
	get [HANDLE](): number {
		if (!this.#alive) throw new NativeError(`${this.constructor.name} was destroyed`, -2);
		return this.#handle;
	}

	/** @internal NativeModule.handleOf 用。 */
	belongsTo(native: NativeModule<string>): boolean {
		return this.native === native;
	}

	get destroyed(): boolean {
		return !this.#alive;
	}

	/** 释放原生对象；重复调用没有效果。 */
	destroy(): void {
		if (!this.#alive) return;
		this.release();
		this.#alive = false;
	}

	[Symbol.dispose](): void {
		this.destroy();
	}

	/** 真正释放原生资源；只调用一次。 */
	protected abstract release(): void;
}
