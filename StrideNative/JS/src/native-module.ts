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

/** 两个模块都有的 C 函数（Core.h）。各模块自己的列表在 *-core.d.mts 里。 */
export type CoreExport =
	| 'sn_init' | 'sn_shutdown' | 'sn_worker_thread_count' | 'sn_last_error' | 'sn_alloc' | 'sn_free'
	| 'sn_task_status_address' | 'sn_task_error' | 'sn_task_result' | 'sn_task_output_count'
	| 'sn_task_output_size' | 'sn_task_output_data' | 'sn_task_release';

/** C 函数在 JS 里的样子：指针、句柄、整数和 float 都是 number（int64 参数会是 bigint，目前没有）。 */
export type NativeFunction = (...args: number[]) => number;

/**
 * Emscripten 编出的模块：C 函数以 `_` 加名字导出，外加我们要用的运行时成员。函数名逐个列出而不用
 * `_sn_${string}` 这样的索引签名：写错名字在编译期就报，而且不会因为 noUncheckedIndexedAccess 变成可能 undefined。
 */
export type CoreModule<E extends string = never> = {
	readonly wasmMemory: WebAssembly.Memory;
	UTF8ToString(ptr: number): string;
} & { readonly [K in CoreExport | E as `_${K}`]: NativeFunction };

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
}

const TASK_RUNNING = 0;
const TASK_DONE = 1;

const isNode = typeof process !== 'undefined' && process.versions?.node != null;

/** 默认开的工作线程数：每个核一个，减去当前线程。 */
export function defaultWorkerThreads(): number {
	const cores = globalThis.navigator?.hardwareConcurrency ?? 4;
	return Math.max(1, cores - 1);
}

/**
 * 实例化 Emscripten 模块并启动任务系统。浏览器里只有跨源隔离的页面（COOP + COEP 响应头）才能用共享内存。
 */
export async function instantiate<E extends string>(factory: CoreFactory<E>, { workerThreads = defaultWorkerThreads(), locateFile }: Omit<ModuleOptions, 'debug'> = {}): Promise<CoreModule<E>> {
	if (!isNode && !globalThis.crossOriginIsolated) {
		throw new NativeError('StrideNative needs a cross-origin isolated page: serve it with ' +
			'"Cross-Origin-Opener-Policy: same-origin" and "Cross-Origin-Embedder-Policy: require-corp"', -4);
	}
	// Worker 池必须装得下任务系统的所有线程：Jolt 是同步创建它们的
	const module = await factory({ workerPoolSize: workerThreads, ...(locateFile ? { locateFile } : {}) });
	const status = module._sn_init(workerThreads);
	if (status !== 0) {
		throw new NativeError(`sn_init failed: ${module.UTF8ToString(module._sn_last_error())}`, status);
	}
	return module;
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
	#shutDown = false;

	constructor(module: CoreModule<E>) {
		this.module = module;
	}

	/** 任务系统的工作线程数（等待任务时当前线程也会帮着跑）。 */
	get workerThreadCount(): number {
		return this.module._sn_worker_thread_count();
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
