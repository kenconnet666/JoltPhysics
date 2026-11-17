// Asynchronous tasks: work queued on the job system, tracked by an id that JS can await.
#pragma once

#include <Core/Core.h>

#include <atomic>
#include <mutex>
#include <vector>

namespace StrideNative
{
	class Task
	{
	public:
		~Task();

		/// Marks the task failed; the first message wins. Safe from any thread.
		void Fail(std::string inMessage);
		bool HasFailed() const { return mFailed.load(std::memory_order_acquire); }

		void SetResult(uint32_t inIndex, int32_t inValue) { mResults[inIndex] = inValue; }

		/// Allocates an output buffer owned by the task. Not thread-safe; parallel items use
		/// SetOutput on slots created by RunParallelTask instead.
		void *AddOutput(uint32_t inSize);
		void *SetOutput(uint32_t inIndex, uint32_t inSize);

	private:
		friend class TaskRegistry;
		friend int32_t RunParallelTask(const char *, uint32_t, std::function<void(Task &, uint32_t)>, std::function<bool(Task &)>, uint32_t);

		struct Output
		{
			void *		mData = nullptr;
			uint32_t	mSize = 0;
		};

		// Read by JS through Atomics, so it must be a plain 32-bit cell.
		alignas(4) std::atomic<int32_t> mStatus { SN_TASK_RUNNING };
		std::atomic<bool>	mFailed { false };
		std::atomic<bool>	mReleaseWhenDone { false };
		std::mutex			mErrorMutex;
		std::string			mError;
		int32_t				mResults[4] = {};
		std::vector<Output>	mOutputs;
	};

	/// Runs inWork(task) on the job system; the task fails if inWork returns false or calls Fail.
	int32_t RunTask(const char *inName, std::function<bool(Task &)> inWork);

	/// Runs inItem(task, index) for every index in parallel jobs, then inFinish(task) once all are
	/// done (skipped if an item failed). inFinish may be empty. inOutputCount output slots are
	/// created up front so items can fill them concurrently with SetOutput.
	int32_t RunParallelTask(const char *inName, uint32_t inCount, std::function<void(Task &, uint32_t)> inItem, std::function<bool(Task &)> inFinish, uint32_t inOutputCount = 0);

	/// Releases every task (module shutdown). Running tasks must have finished.
	void ReleaseAllTasks();
}
