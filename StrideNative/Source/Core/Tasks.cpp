#include <Core/Tasks.h>
#include <Core/HandleTable.h>

#include <algorithm>

#ifdef __EMSCRIPTEN__
	#include <emscripten/atomic.h>
#endif

namespace StrideNative
{
	class TaskRegistry
	{
	public:
		static HandleTable<Task> &Tasks()
		{
			static HandleTable<Task> sTasks;
			return sTasks;
		}

		static Task *Get(int32_t inTask)
		{
			Task *task = Tasks().Get(inTask);
			if (task == nullptr)
				SetLastError(SN_ERROR_INVALID_HANDLE, "unknown task " + std::to_string(inTask));
			return task;
		}

		static int32_t Add(std::unique_ptr<Task> inTask)
		{
			return Tasks().Add(std::move(inTask));
		}

		// Completion and release race when JS releases a running task; one lock orders them.
		static void Complete(int32_t inId, Task &ioTask)
		{
			std::lock_guard lock(sCompletionMutex);
			ioTask.mStatus.store(ioTask.HasFailed() ? SN_TASK_FAILED : SN_TASK_DONE, std::memory_order_release);
#ifdef __EMSCRIPTEN__
			emscripten_atomic_notify(&ioTask.mStatus, EMSCRIPTEN_NOTIFY_ALL_WAITERS);
#endif
			if (ioTask.mReleaseWhenDone.load(std::memory_order_relaxed))
				Tasks().Remove(inId);
		}

		static void Release(int32_t inId)
		{
			std::lock_guard lock(sCompletionMutex);
			Task *task = Tasks().Get(inId);
			if (task == nullptr)
				return;
			if (task->mStatus.load(std::memory_order_acquire) == SN_TASK_RUNNING)
				task->mReleaseWhenDone.store(true, std::memory_order_relaxed);
			else
				Tasks().Remove(inId);
		}

		static const int32_t *StatusAddress(Task &inTask)
		{
			static_assert(sizeof(std::atomic<int32_t>) == sizeof(int32_t), "status must be a plain int32 for Atomics");
			return reinterpret_cast<const int32_t *>(&inTask.mStatus);
		}

		static int32_t Status(Task &inTask) { return inTask.mStatus.load(std::memory_order_acquire); }
		static const char *Error(Task &inTask) { return inTask.mError.c_str(); }
		static int32_t Result(Task &inTask, uint32_t inIndex) { return inIndex < 4 ? inTask.mResults[inIndex] : 0; }
		static const std::vector<Task::Output> &Outputs(Task &inTask) { return inTask.mOutputs; }

	private:
		static inline std::mutex sCompletionMutex;
	};

	Task::~Task()
	{
		for (Output &output : mOutputs)
			if (output.mData != nullptr)
				JPH::AlignedFree(output.mData);
	}

	void Task::Fail(std::string inMessage)
	{
		std::lock_guard lock(mErrorMutex);
		if (mFailed.load(std::memory_order_relaxed))
			return;
		mError = std::move(inMessage);
		mFailed.store(true, std::memory_order_release);
	}

	void *Task::AddOutput(uint32_t inSize)
	{
		mOutputs.emplace_back();
		return SetOutput(uint32_t(mOutputs.size() - 1), inSize);
	}

	void *Task::SetOutput(uint32_t inIndex, uint32_t inSize)
	{
		Output &output = mOutputs[inIndex];
		if (output.mData != nullptr)
			JPH::AlignedFree(output.mData);
		output.mData = inSize > 0 ? JPH::AlignedAllocate(inSize, 16) : nullptr;
		output.mSize = inSize;
		return output.mData;
	}

	int32_t RunTask(const char *inName, std::function<bool(Task &)> inWork)
	{
		auto owned = std::make_unique<Task>();
		Task *task = owned.get();
		int32_t id = TaskRegistry::Add(std::move(owned));
		GetJobSystem().CreateJob(inName, JPH::Color::sGrey, [id, task, work = std::move(inWork)]()
		{
			if (!work(*task))
				task->Fail("operation failed");
			TaskRegistry::Complete(id, *task);
		});
		return id;
	}

	int32_t RunParallelTask(const char *inName, uint32_t inCount, std::function<void(Task &, uint32_t)> inItem, std::function<bool(Task &)> inFinish, uint32_t inOutputCount)
	{
		auto owned = std::make_unique<Task>();
		owned->mOutputs.resize(inOutputCount);
		Task *task = owned.get();
		int32_t id = TaskRegistry::Add(std::move(owned));

		JPH::JobSystem &job_system = GetJobSystem();
		// A few chunks per thread keeps all threads busy without exhausting the job pool.
		uint32_t chunk_count = std::max(1u, std::min(inCount, uint32_t(job_system.GetMaxConcurrency()) * 4));
		JPH::JobHandle finish = job_system.CreateJob(inName, JPH::Color::sGrey, [id, task, finish_work = std::move(inFinish)]()
		{
			if (!task->HasFailed() && finish_work && !finish_work(*task))
				task->Fail("operation failed");
			TaskRegistry::Complete(id, *task);
		}, chunk_count);

		auto item = std::make_shared<std::function<void(Task &, uint32_t)>>(std::move(inItem));
		for (uint32_t chunk = 0; chunk < chunk_count; ++chunk)
		{
			uint32_t begin = uint32_t(uint64_t(inCount) * chunk / chunk_count);
			uint32_t end = uint32_t(uint64_t(inCount) * (chunk + 1) / chunk_count);
			job_system.CreateJob(inName, JPH::Color::sGrey, [task, item, finish, begin, end]()
			{
				for (uint32_t index = begin; index < end && !task->HasFailed(); ++index)
					(*item)(*task, index);
				finish.RemoveDependency();
			});
		}
		return id;
	}

	void ReleaseAllTasks()
	{
		TaskRegistry::Tasks().Clear();
	}
}

using namespace StrideNative;

SN_API int32_t sn_task_status(int32_t task)
{
	Task *t = TaskRegistry::Get(task);
	return t != nullptr ? TaskRegistry::Status(*t) : SN_ERROR_INVALID_HANDLE;
}

SN_API const int32_t *sn_task_status_address(int32_t task)
{
	Task *t = TaskRegistry::Get(task);
	return t != nullptr ? TaskRegistry::StatusAddress(*t) : nullptr;
}

SN_API const char *sn_task_error(int32_t task)
{
	Task *t = TaskRegistry::Get(task);
	return t != nullptr ? TaskRegistry::Error(*t) : "unknown task";
}

SN_API int32_t sn_task_result(int32_t task, uint32_t index)
{
	Task *t = TaskRegistry::Get(task);
	return t != nullptr ? TaskRegistry::Result(*t, index) : 0;
}

SN_API uint32_t sn_task_output_count(int32_t task)
{
	Task *t = TaskRegistry::Get(task);
	return t != nullptr ? uint32_t(TaskRegistry::Outputs(*t).size()) : 0;
}

SN_API void *sn_task_output_data(int32_t task, uint32_t index)
{
	Task *t = TaskRegistry::Get(task);
	if (t == nullptr || index >= TaskRegistry::Outputs(*t).size())
		return nullptr;
	return TaskRegistry::Outputs(*t)[index].mData;
}

SN_API uint32_t sn_task_output_size(int32_t task, uint32_t index)
{
	Task *t = TaskRegistry::Get(task);
	if (t == nullptr || index >= TaskRegistry::Outputs(*t).size())
		return 0;
	return TaskRegistry::Outputs(*t)[index].mSize;
}

SN_API void sn_task_release(int32_t task)
{
	TaskRegistry::Release(task);
}
