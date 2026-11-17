// Internal core services shared by every module: the job system, errors and allocation.
#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystem.h>
#include <Jolt/Core/TempAllocator.h>

#include <StrideNative/Core.h>

#include <functional>
#include <string>

namespace StrideNative
{
	bool IsInitialized();

	/// The one thread pool of the module: physics steps and background tasks share it.
	JPH::JobSystem &GetJobSystem();
	JPH::TempAllocator &GetTempAllocator();

	/// Records the message returned by sn_last_error() on this thread and returns inCode.
	int32_t SetLastError(int32_t inCode, std::string inMessage);

	/// Runs inHook during sn_shutdown, before the job system and Jolt are torn down (last added
	/// runs first). Modules use it to destroy the objects their handle tables still own. Hooks stay
	/// registered across init/shutdown cycles, so register once (e.g. on first use of a table).
	void AddShutdownHook(std::function<void()> inHook);

	/// Runs inBody(begin, end) over [0, inCount) split into chunks on the job system and waits;
	/// the calling thread executes chunks while it waits, so this is safe on the browser main thread.
	void ParallelFor(uint32_t inCount, const std::function<void(uint32_t inBegin, uint32_t inEnd)> &inBody);
}

/// Shorthand for the guard every exported function starts with.
#define SN_REQUIRE_INIT(...)																	\
	do																							\
	{																							\
		if (!StrideNative::IsInitialized())														\
		{																						\
			StrideNative::SetLastError(SN_ERROR_NOT_INITIALIZED, "sn_init has not been called");	\
			return __VA_ARGS__;																	\
		}																						\
	} while (false)
