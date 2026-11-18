// StrideNative debug instrumentation: named counters, timings and log lines that JS reads for the
// stride-ts debug CLI (`stride native`, `stride logs`). Every module exports these (they live in the
// core), and plugins can call them too.
//
// Collection is off by default and then costs one relaxed atomic load per call site; JS turns it on
// while a debugger is attached (sn_debug_set_enabled). Names are copied, so they can be temporary.
#pragma once

#include <StrideNative/Core.h>

enum
{
	SN_LOG_DEBUG = 0,
	SN_LOG_INFO = 1,
	SN_LOG_WARN = 2,
	SN_LOG_ERROR = 3,
};

SN_API void sn_debug_set_enabled(int32_t enabled);
SN_API int32_t sn_debug_enabled(void);

/// Sets a counter to its current value, e.g. "physics.bodies".
SN_API void sn_debug_set_counter(const char *name, double value);
/// Adds one call that took `milliseconds` to a timing, e.g. "physics.step".
SN_API void sn_debug_add_timing(const char *name, double milliseconds);
/// Queues a log line for JS. While collection is off only SN_LOG_ERROR lines are queued, so an error
/// before the debugger attaches is not lost. The queue is bounded (oldest lines are dropped).
SN_API void sn_debug_log(int32_t level, const char *message);

/// JSON object {"counters":{name:value},"timings":{name:{"calls":n,"ms":total,"maxMs":m}},"memory":{...}}.
/// The string stays valid until the next call on the calling thread.
SN_API const char *sn_debug_snapshot(void);
/// Clears the timings (counters keep their last values).
SN_API void sn_debug_reset_timings(void);
/// Queued log lines as a JSON array [{"level":n,"message":"..."}], removed from the queue. The string
/// stays valid until the next call on the calling thread.
SN_API const char *sn_debug_drain_log(void);
