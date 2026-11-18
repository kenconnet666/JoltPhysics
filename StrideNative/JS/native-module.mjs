// Shared plumbing for the StrideNative wasm modules: memory access, struct marshalling, errors and
// awaiting native tasks. Both modules are multithreaded, so their memory is a SharedArrayBuffer.

/** Error raised by a failed native call; `code` is the SN_ERROR_* value (negative). */
export class NativeError extends Error {
	constructor(message, code) {
		super(message);
		this.name = 'NativeError';
		this.code = code;
	}
}

const TASK_RUNNING = 0;
const TASK_DONE = 1;

const isNode = typeof process !== 'undefined' && process.versions?.node != null;

/** Number of worker threads to start when the caller does not say: one per core, minus this thread. */
export function defaultWorkerThreads() {
	const cores = globalThis.navigator?.hardwareConcurrency ?? 4;
	return Math.max(1, cores - 1);
}

/**
 * Instantiates an Emscripten module factory and starts its job system.
 * Browsers only allow shared memory on cross-origin isolated pages (COOP + COEP headers).
 */
export async function instantiate(factory, { workerThreads = defaultWorkerThreads(), locateFile } = {}) {
	if (!isNode && !globalThis.crossOriginIsolated) {
		throw new NativeError('StrideNative needs a cross-origin isolated page: serve it with ' +
			'"Cross-Origin-Opener-Policy: same-origin" and "Cross-Origin-Embedder-Policy: require-corp"', -4);
	}
	// The Worker pool must hold every job-system thread: Jolt creates them synchronously.
	const module = await factory({ workerPoolSize: workerThreads, ...(locateFile ? { locateFile } : {}) });
	const code = module._sn_init(workerThreads);
	if (code !== 0) {
		throw new NativeError(`sn_init failed: ${module.UTF8ToString(module._sn_last_error())}`, code);
	}
	return module;
}

/** Base class of the module wrappers. */
export class NativeModule {
	constructor(module) {
		this.module = module;
	}

	/** Worker threads of the job system (the calling thread also runs jobs while it waits). */
	get workerThreadCount() {
		return this.module._sn_worker_thread_count();
	}

	/** Stops the job system and frees every native object. The module is unusable afterwards. */
	shutdown() {
		this.module._sn_shutdown();
	}

	// Views are created on demand: after the shared memory grows, memory.buffer is a larger buffer.
	get buffer() { return this.module.wasmMemory.buffer; }
	u8(ptr, length) { return new Uint8Array(this.buffer, ptr, length); }
	i32(ptr, length) { return new Int32Array(this.buffer, ptr, length); }
	u32(ptr, length) { return new Uint32Array(this.buffer, ptr, length); }
	f32(ptr, length) { return new Float32Array(this.buffer, ptr, length); }
	f64(ptr, length) { return new Float64Array(this.buffer, ptr, length); }
	// int64 / uint64 的视图：stride-ts 的 i64、decimal、time、datetime、instant 列和 C 的 sn_* 值类型（Values.h）共用这个布局
	i64(ptr, length) { return new BigInt64Array(this.buffer, ptr, length); }
	u64(ptr, length) { return new BigUint64Array(this.buffer, ptr, length); }
	view(ptr, length) { return new DataView(this.buffer, ptr, length); }

	alloc(size) {
		const ptr = this.module._sn_alloc(size);
		if (size > 0 && ptr === 0) {
			throw new NativeError(`allocating ${size} bytes failed`, -3);
		}
		return ptr;
	}

	free(ptr) {
		this.module._sn_free(ptr);
	}

	/** Copies a typed array into a new native allocation and returns its pointer. */
	copyIn(array) {
		const bytes = new Uint8Array(array.buffer, array.byteOffset, array.byteLength);
		const ptr = this.alloc(bytes.byteLength);
		this.u8(ptr, bytes.byteLength).set(bytes);
		return ptr;
	}

	/** Copies native memory into a new, ordinary (non-shared) typed array of the given type. */
	copyOut(ArrayType, ptr, length) {
		return new ArrayType(this.buffer, ptr, length).slice();
	}

	/** Runs fn(...pointers) with the arrays copied into native memory, then frees them. Arrays may be null. */
	withArrays(arrays, fn) {
		const pointers = arrays.map((array) => (array ? this.copyIn(array) : 0));
		try {
			return fn(...pointers);
		} finally {
			pointers.forEach((ptr) => ptr && this.free(ptr));
		}
	}

	/** Async variant of withArrays: the arrays stay allocated until the returned promise settles. */
	async withArraysAsync(arrays, fn) {
		const pointers = arrays.map((array) => (array ? this.copyIn(array) : 0));
		try {
			return await fn(...pointers);
		} finally {
			pointers.forEach((ptr) => ptr && this.free(ptr));
		}
	}

	/** Allocates a scratch block, passes it to fn, frees it. */
	withScratch(size, fn) {
		const ptr = this.alloc(size);
		try {
			return fn(ptr);
		} finally {
			this.free(ptr);
		}
	}

	lastError() {
		return this.module.UTF8ToString(this.module._sn_last_error());
	}

	/** Throws if a native call returned a negative status; returns the value otherwise. */
	check(value, what) {
		if (value < 0) {
			throw new NativeError(`${what}: ${this.lastError()}`, value);
		}
		return value;
	}

	/**
	 * Waits for a native task without blocking the thread, reads it with read(task) and releases it.
	 * Uses Atomics.waitAsync on the task's status cell (the module notifies it on completion) and falls
	 * back to polling where waitAsync is missing.
	 */
	async awaitTask(task, what, read = () => undefined) {
		this.check(task, what);
		const m = this.module;
		// In Node neither a pending Atomics.waitAsync nor Emscripten's (unref'd) workers keep the
		// process alive, so it could exit mid-task; a timer holds the event loop open while we wait.
		const keepAlive = isNode ? setInterval(() => {}, 1 << 30) : null;
		try {
			const statusPtr = m._sn_task_status_address(task);
			const status = this.i32(statusPtr, 1);
			while (Atomics.load(status, 0) === TASK_RUNNING) {
				if (typeof Atomics.waitAsync === 'function') {
					const wait = Atomics.waitAsync(status, 0, TASK_RUNNING);
					if (wait.async) {
						await wait.value;
					}
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
				output: (ArrayType, index) => {
					const size = m._sn_task_output_size(task, index);
					return this.copyOut(ArrayType, m._sn_task_output_data(task, index), size / ArrayType.BYTES_PER_ELEMENT);
				},
			});
		} finally {
			if (keepAlive) {
				clearInterval(keepAlive);
			}
			m._sn_task_release(task);
		}
	}

	/** Reads a buffer that a native call allocated through an out pointer pair, then frees it. */
	readOwnedBuffer(call, what) {
		return this.withScratch(8, (out) => {
			this.check(call(out, out + 4), what);
			const [dataPtr, size] = this.u32(out, 2);
			const bytes = this.copyOut(Uint8Array, dataPtr, size);
			this.free(dataPtr);
			return bytes;
		});
	}
}

/** Writes a sn_nav_config: native defaults first, then the given fields (camelCase names). */
export function writeNavConfig(native, ptr, config = {}) {
	native.module._sn_nav_default_config(ptr);
	const fields = [
		['cellSize', 'f'], ['cellHeight', 'f'], ['agentHeight', 'f'], ['agentRadius', 'f'],
		['agentMaxClimb', 'f'], ['agentMaxSlopeDegrees', 'f'], ['tileSize', 'i'], ['maxEdgeLength', 'f'],
		['maxSimplificationError', 'f'], ['minRegionSize', 'f'], ['mergeRegionSize', 'f'],
		['detailSampleDistance', 'f'], ['detailSampleMaxError', 'f'],
	];
	const view = native.view(ptr, fields.length * 4);
	fields.forEach(([name, type], index) => {
		if (config[name] === undefined) {
			return;
		}
		if (type === 'i') {
			view.setInt32(index * 4, config[name], true);
		} else {
			view.setFloat32(index * 4, config[name], true);
		}
	});
}

export const NAV_CONFIG_SIZE = 13 * 4;
