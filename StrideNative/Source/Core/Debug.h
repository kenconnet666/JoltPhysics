// C++ helpers over the sn_debug_* API (Include/StrideNative/Debug.h).
#pragma once

#include <StrideNative/Debug.h>

#include <atomic>
#include <chrono>

namespace StrideNative
{
	/// True while JS collects debug data; check it before computing anything only a counter needs.
	bool DebugEnabled();

	/// printf-style sn_debug_log.
	void DebugLog(int32_t inLevel, const char *inFormat, ...)
#if defined(__GNUC__) || defined(__clang__)
		__attribute__((format(printf, 2, 3)))
#endif
		;

	/// Adds the time from construction to destruction to a timing; does nothing while collection is off.
	class DebugScope
	{
	public:
		explicit DebugScope(const char *inName) :
			mName(DebugEnabled() ? inName : nullptr)
		{
			if (mName != nullptr)
				mStart = std::chrono::steady_clock::now();
		}

		~DebugScope()
		{
			if (mName != nullptr)
				sn_debug_add_timing(mName, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - mStart).count());
		}

		DebugScope(const DebugScope &) = delete;
		DebugScope &operator=(const DebugScope &) = delete;

	private:
		const char *mName;
		std::chrono::steady_clock::time_point mStart;
	};
}

#define SN_DEBUG_CONCAT_INNER(a, b) a##b
#define SN_DEBUG_CONCAT(a, b) SN_DEBUG_CONCAT_INNER(a, b)
/// Times the rest of the enclosing block under inName (a string literal).
#define SN_DEBUG_SCOPE(inName) ::StrideNative::DebugScope SN_DEBUG_CONCAT(snDebugScope, __LINE__)(inName)
