// Helpers shared by the native tests. The JS tests in Tests/JS cover the same API through wasm.
#pragma once

#include <StrideNative/Core.h>

#include <doctest.h>

#include <chrono>
#include <thread>

/// Starts the module for one test and shuts it down afterwards.
struct ModuleScope
{
	ModuleScope() { REQUIRE(sn_init(-1) == SN_OK); }
	~ModuleScope() { sn_shutdown(); }
};

/// Blocks until the task finishes and returns its final status.
inline int32_t WaitForTask(int32_t inTask)
{
	REQUIRE(inTask > 0);
	while (sn_task_status(inTask) == SN_TASK_RUNNING)
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	int32_t status = sn_task_status(inTask);
	if (status != SN_TASK_DONE)
		MESSAGE("task failed: " << sn_task_error(inTask));
	return status;
}
