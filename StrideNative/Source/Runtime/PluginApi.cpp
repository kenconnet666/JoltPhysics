// 插件 C API（Plugin.h）：把核心的任务系统、错误和关机钩子包成 C 函数给第三方侧模块用。
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Plugin.h>

using namespace StrideNative;

// 不透明类型：插件拿到的 sn_task_context* 就是 Task*
struct sn_task_context;

namespace
{
	Task &ToTask(sn_task_context *inTask)
	{
		return *reinterpret_cast<Task *>(inTask);
	}
}

SN_API uint32_t sn_plugin_abi_version(void)
{
	return SN_PLUGIN_ABI_VERSION;
}

SN_API int32_t sn_set_last_error(int32_t code, const char *message)
{
	return SetLastError(code, message != nullptr ? message : "");
}

SN_API void sn_parallel_for(uint32_t count, sn_parallel_fn fn, void *user)
{
	SN_REQUIRE_INIT();
	ParallelFor(count, [fn, user](uint32_t inBegin, uint32_t inEnd) { fn(user, inBegin, inEnd); });
}

SN_API int32_t sn_run_task(const char *name, sn_task_fn fn, void *user, void (*free_user)(void *user))
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (fn == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_run_task: fn is null");
	return RunTask(name != nullptr ? name : "PluginTask", [fn, user, free_user](Task &ioTask)
	{
		int32_t status = fn(user, reinterpret_cast<sn_task_context *>(&ioTask));
		if (free_user != nullptr)
			free_user(user);
		return status == 0;
	});
}

SN_API void sn_task_set_result(sn_task_context *task, uint32_t index, int32_t value)
{
	if (index < 4)
		ToTask(task).SetResult(index, value);
}

SN_API void *sn_task_add_output(sn_task_context *task, uint32_t size)
{
	return ToTask(task).AddOutput(size);
}

SN_API void sn_task_fail(sn_task_context *task, const char *message)
{
	ToTask(task).Fail(message != nullptr ? message : "operation failed");
}

SN_API void sn_on_shutdown(void (*fn)(void *user), void *user)
{
	if (fn != nullptr)
		AddShutdownHook([fn, user] { fn(user); });
}
