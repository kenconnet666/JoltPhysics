// StrideNative core C API: lifetime, memory, errors and asynchronous tasks.
// Every module (stride-native, stride-tools) exports these. All sizes are uint32_t because the
// modules are 32-bit wasm; pointers passed from JS are offsets into the module's memory.
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __EMSCRIPTEN__
	#include <emscripten/emscripten.h>
	// default 可见性：拆分构建的侧模块用 -fvisibility=hidden 编译，只导出 C API
	#define SN_EXPORT EMSCRIPTEN_KEEPALIVE __attribute__((visibility("default")))
#else
	#define SN_EXPORT
#endif

#ifdef __cplusplus
	#define SN_API extern "C" SN_EXPORT
#else
	#define SN_API SN_EXPORT
#endif

enum
{
	SN_OK = 0,
	SN_ERROR_INVALID_ARGUMENT = -1,
	SN_ERROR_INVALID_HANDLE = -2,
	SN_ERROR_FAILED = -3,
	SN_ERROR_NOT_INITIALIZED = -4,
};

/// Starts the shared job system. worker_threads < 0 uses (hardware threads - 1); 0 is rejected because
/// tasks only run on worker threads. Call once.
SN_API int32_t sn_init(int32_t worker_threads);
SN_API void sn_shutdown(void);
SN_API int32_t sn_worker_thread_count(void);

/// Message of the last failed call on the calling thread, or "" if none.
SN_API const char *sn_last_error(void);

/// 16-byte aligned allocation in module memory; free with sn_free.
SN_API void *sn_alloc(uint32_t size);
SN_API void sn_free(void *ptr);

// Tasks: long operations (decoding, building, encoding) run on the job system and are tracked by a
// task id. The status lives in shared memory so JS can await it with Atomics.waitAsync; the module
// calls memory.atomic.notify on it when the task finishes.
enum
{
	SN_TASK_RUNNING = 0,
	SN_TASK_DONE = 1,
	SN_TASK_FAILED = 2,
};

SN_API int32_t sn_task_status(int32_t task);
SN_API const int32_t *sn_task_status_address(int32_t task);
SN_API const char *sn_task_error(int32_t task);
/// Number of small integer results a task has (sn_task_result).
#define SN_TASK_RESULT_COUNT 8
/// Small integer results (handles, counts, dimensions); meaning depends on the operation.
/// index is 0..SN_TASK_RESULT_COUNT-1; other indices read 0.
SN_API int32_t sn_task_result(int32_t task, uint32_t index);
/// Output buffers produced by the task. They stay valid until sn_task_release.
SN_API uint32_t sn_task_output_count(int32_t task);
SN_API void *sn_task_output_data(int32_t task, uint32_t index);
SN_API uint32_t sn_task_output_size(int32_t task, uint32_t index);
/// Frees the task and its outputs. A running task is released as soon as it finishes.
SN_API void sn_task_release(int32_t task);
