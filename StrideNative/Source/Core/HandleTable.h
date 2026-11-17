// Maps positive int32 handles (what JS holds) to owned C++ objects.
#pragma once

#include <Core/Core.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace StrideNative
{
	/// Thread-safe: tasks create objects on worker threads while JS looks them up on the main thread.
	/// Slots are reused, so a stale handle may hit a newer object; callers own handle lifetimes.
	template <class T>
	class HandleTable
	{
	public:
		int32_t Add(std::unique_ptr<T> inObject)
		{
			std::lock_guard lock(mMutex);
			if (!mFreeSlots.empty())
			{
				uint32_t slot = mFreeSlots.back();
				mFreeSlots.pop_back();
				mObjects[slot] = std::move(inObject);
				return int32_t(slot + 1);
			}
			mObjects.push_back(std::move(inObject));
			return int32_t(mObjects.size());
		}

		/// Returns nullptr for unknown handles.
		T *Get(int32_t inHandle)
		{
			std::lock_guard lock(mMutex);
			if (inHandle <= 0 || uint32_t(inHandle) > mObjects.size())
				return nullptr;
			return mObjects[inHandle - 1].get();
		}

		std::unique_ptr<T> Remove(int32_t inHandle)
		{
			std::lock_guard lock(mMutex);
			if (inHandle <= 0 || uint32_t(inHandle) > mObjects.size() || mObjects[inHandle - 1] == nullptr)
				return nullptr;
			mFreeSlots.push_back(uint32_t(inHandle - 1));
			return std::move(mObjects[inHandle - 1]);
		}

		void Clear()
		{
			std::lock_guard lock(mMutex);
			mObjects.clear();
			mFreeSlots.clear();
		}

	private:
		std::mutex mMutex;
		std::vector<std::unique_ptr<T>> mObjects;
		std::vector<uint32_t> mFreeSlots;
	};

	/// The module-wide table for T; its objects are destroyed on sn_shutdown.
	template <class T>
	HandleTable<T> &GetTable()
	{
		static HandleTable<T> sTable;
		static const bool sHooked = (AddShutdownHook([] { sTable.Clear(); }), true);
		(void)sHooked;
		return sTable;
	}

	/// Looks up a handle and records an error naming inWhat when it is unknown.
	template <class T>
	T *Lookup(int32_t inHandle, const char *inWhat)
	{
		T *object = GetTable<T>().Get(inHandle);
		if (object == nullptr)
			SetLastError(SN_ERROR_INVALID_HANDLE, std::string("unknown ") + inWhat + " " + std::to_string(inHandle));
		return object;
	}
}
