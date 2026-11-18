#include <Core/Core.h>
#include <Core/Debug.h>
#include <Core/Tasks.h>

#include <Jolt/Core/JobSystemThreadPool.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#endif

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <vector>

namespace
{
	// Physics uses one barrier per step; background tasks and ParallelFor use the rest.
	constexpr JPH::uint cMaxJobs = 4096;
	constexpr JPH::uint cMaxBarriers = 64;
	constexpr JPH::uint cTempAllocatorSize = 32 * 1024 * 1024;

	std::unique_ptr<JPH::JobSystemThreadPool> sJobSystem;
	std::unique_ptr<JPH::TempAllocatorImpl> sTempAllocator;
	std::vector<std::function<void()>> sShutdownHooks;
	thread_local std::string tLastError;

	void TraceImpl(const char *inFormat, ...)
	{
		char message[1024];
		va_list list;
		va_start(list, inFormat);
		std::vsnprintf(message, sizeof(message), inFormat, list);
		va_end(list);
		std::fprintf(stderr, "%s\n", message);
		// 调试 CLI 连着时也转给 JS（stride logs 里 source 是 native）
		sn_debug_log(SN_LOG_INFO, message);
	}

#ifdef JPH_ENABLE_ASSERTS
	bool AssertFailedImpl(const char *inExpression, const char *inMessage, const char *inFile, JPH::uint inLine)
	{
		StrideNative::DebugLog(SN_LOG_ERROR, "assert failed: %s:%u: (%s) %s", inFile, inLine, inExpression, inMessage != nullptr ? inMessage : "");
#ifdef __EMSCRIPTEN__
		// 浏览器里打到 console.error，带上调用栈（wasm 留着函数名，栈里是 C++ 的函数）
		emscripten_log(EM_LOG_CONSOLE | EM_LOG_ERROR | EM_LOG_JS_STACK, "assert failed: %s:%u: (%s) %s", inFile, inLine, inExpression, inMessage != nullptr ? inMessage : "");
#else
		std::fprintf(stderr, "%s:%u: (%s) %s\n", inFile, inLine, inExpression, inMessage != nullptr ? inMessage : "");
#endif
		return true; // Break: an assert in our code or Jolt's means corrupted state
	}
#endif
}

namespace StrideNative
{
	bool IsInitialized()
	{
		return sJobSystem != nullptr;
	}

	JPH::JobSystem &GetJobSystem()
	{
		return *sJobSystem;
	}

	JPH::TempAllocator &GetTempAllocator()
	{
		return *sTempAllocator;
	}

	int32_t SetLastError(int32_t inCode, std::string inMessage)
	{
		tLastError = std::move(inMessage);
		return inCode;
	}

	void AddShutdownHook(std::function<void()> inHook)
	{
		sShutdownHooks.push_back(std::move(inHook));
	}

	void ParallelFor(uint32_t inCount, const std::function<void(uint32_t, uint32_t)> &inBody)
	{
		JPH::JobSystem &job_system = GetJobSystem();
		uint32_t chunk_count = std::min(inCount, uint32_t(job_system.GetMaxConcurrency()) * 2);
		if (chunk_count <= 1)
		{
			if (inCount > 0)
				inBody(0, inCount);
			return;
		}

		JPH::JobSystem::Barrier *barrier = job_system.CreateBarrier();
		for (uint32_t chunk = 0; chunk < chunk_count; ++chunk)
		{
			uint32_t begin = uint32_t(uint64_t(inCount) * chunk / chunk_count);
			uint32_t end = uint32_t(uint64_t(inCount) * (chunk + 1) / chunk_count);
			barrier->AddJob(job_system.CreateJob("ParallelFor", JPH::Color::sGrey, [&inBody, begin, end]() { inBody(begin, end); }));
		}
		job_system.WaitForJobs(barrier);
		job_system.DestroyBarrier(barrier);
	}
}

using namespace StrideNative;

SN_API int32_t sn_init(int32_t worker_threads)
{
	if (IsInitialized())
		return SetLastError(SN_ERROR_FAILED, "sn_init was already called");
	// Tasks run only on workers (a waiting thread helps only with barrier jobs), so 0 would hang them.
	if (worker_threads == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "at least one worker thread is required");

	JPH::RegisterDefaultAllocator();
	JPH::Trace = TraceImpl;
	JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)
	// Jolt 的类型（形状、约束……）由物理模块在第一次创建世界时注册，见 PhysicsApi.cpp：
	// 物理是单独的侧模块（wasm 里），核心不能依赖它

	sTempAllocator = std::make_unique<JPH::TempAllocatorImpl>(cTempAllocatorSize);
	sJobSystem = std::make_unique<JPH::JobSystemThreadPool>(cMaxJobs, cMaxBarriers, worker_threads);
	return SN_OK;
}

SN_API void sn_shutdown(void)
{
	if (!IsInitialized())
		return;

	for (auto hook = sShutdownHooks.rbegin(); hook != sShutdownHooks.rend(); ++hook)
		(*hook)();
	ReleaseAllTasks();

	sJobSystem.reset();
	sTempAllocator.reset();
}

SN_API int32_t sn_worker_thread_count(void)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	return sJobSystem->GetMaxConcurrency() - 1;
}

SN_API const char *sn_last_error(void)
{
	return tLastError.c_str();
}

SN_API void *sn_alloc(uint32_t size)
{
	SN_REQUIRE_INIT(nullptr);
	return size > 0 ? JPH::AlignedAllocate(size, 16) : nullptr;
}

SN_API void sn_free(void *ptr)
{
	if (ptr != nullptr)
		JPH::AlignedFree(ptr);
}
