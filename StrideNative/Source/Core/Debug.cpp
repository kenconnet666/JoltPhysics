#include <Core/Debug.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/heap.h>
#include <malloc.h>
#endif

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <map>
#include <mutex>
#include <string>

namespace
{
	struct Timing
	{
		uint64_t mCalls = 0;
		double mTotal = 0;
		double mMax = 0;
	};

	struct LogLine
	{
		int32_t mLevel;
		std::string mMessage;
	};

	/// Lines kept for JS at most; older ones are dropped.
	constexpr size_t cMaxLogLines = 1000;

	std::atomic<bool> sEnabled { false };
	std::mutex sMutex;
	std::map<std::string, double> sCounters;
	std::map<std::string, Timing> sTimings;
	std::deque<LogLine> sLog;
	thread_local std::string tOutput;

	void AppendString(std::string &ioOut, const char *inText)
	{
		ioOut += '"';
		for (const char *c = inText; *c != 0; ++c)
		{
			switch (*c)
			{
			case '"': ioOut += "\\\""; break;
			case '\\': ioOut += "\\\\"; break;
			case '\n': ioOut += "\\n"; break;
			case '\r': ioOut += "\\r"; break;
			case '\t': ioOut += "\\t"; break;
			default:
				if (static_cast<unsigned char>(*c) < 0x20)
				{
					char escaped[8];
					std::snprintf(escaped, sizeof(escaped), "\\u%04x", *c);
					ioOut += escaped;
				}
				else
					ioOut += *c;
			}
		}
		ioOut += '"';
	}

	void AppendNumber(std::string &ioOut, double inValue)
	{
		if (!std::isfinite(inValue))
		{
			ioOut += "null";
			return;
		}
		char number[32];
		std::snprintf(number, sizeof(number), "%.17g", inValue);
		ioOut += number;
	}
}

namespace StrideNative
{
	bool DebugEnabled()
	{
		return sEnabled.load(std::memory_order_relaxed);
	}

	void DebugLog(int32_t inLevel, const char *inFormat, ...)
	{
		if (!DebugEnabled() && inLevel < SN_LOG_ERROR)
			return;
		char message[1024];
		va_list list;
		va_start(list, inFormat);
		std::vsnprintf(message, sizeof(message), inFormat, list);
		va_end(list);
		sn_debug_log(inLevel, message);
	}
}

using namespace StrideNative;

SN_API void sn_debug_set_enabled(int32_t enabled)
{
	sEnabled.store(enabled != 0, std::memory_order_relaxed);
}

SN_API int32_t sn_debug_enabled(void)
{
	return DebugEnabled() ? 1 : 0;
}

SN_API void sn_debug_set_counter(const char *name, double value)
{
	if (!DebugEnabled() || name == nullptr)
		return;
	std::lock_guard lock(sMutex);
	sCounters[name] = value;
}

SN_API void sn_debug_add_timing(const char *name, double milliseconds)
{
	if (!DebugEnabled() || name == nullptr)
		return;
	std::lock_guard lock(sMutex);
	Timing &timing = sTimings[name];
	timing.mCalls++;
	timing.mTotal += milliseconds;
	if (milliseconds > timing.mMax)
		timing.mMax = milliseconds;
}

SN_API void sn_debug_log(int32_t level, const char *message)
{
	if ((!DebugEnabled() && level < SN_LOG_ERROR) || message == nullptr)
		return;
	std::lock_guard lock(sMutex);
	if (sLog.size() >= cMaxLogLines)
		sLog.pop_front();
	sLog.push_back({ level, message });
}

SN_API const char *sn_debug_snapshot(void)
{
	std::string &out = tOutput;
	out = "{\"counters\":{";
	{
		std::lock_guard lock(sMutex);
		bool first = true;
		for (const auto &[name, value] : sCounters)
		{
			if (!first)
				out += ',';
			first = false;
			AppendString(out, name.c_str());
			out += ':';
			AppendNumber(out, value);
		}
		out += "},\"timings\":{";
		first = true;
		for (const auto &[name, timing] : sTimings)
		{
			if (!first)
				out += ',';
			first = false;
			AppendString(out, name.c_str());
			out += ":{\"calls\":";
			AppendNumber(out, double(timing.mCalls));
			out += ",\"ms\":";
			AppendNumber(out, timing.mTotal);
			out += ",\"maxMs\":";
			AppendNumber(out, timing.mMax);
			out += '}';
		}
	}
	out += "},\"memory\":{";
#ifdef __EMSCRIPTEN__
	// 堆的大小（wasm 内存）和 malloc 实际分出去的字节
	struct mallinfo info = mallinfo();
	out += "\"heapBytes\":";
	AppendNumber(out, double(emscripten_get_heap_size()));
	out += ",\"allocatedBytes\":";
	AppendNumber(out, double(info.uordblks));
#endif
	out += "}}";
	return out.c_str();
}

SN_API void sn_debug_reset_timings(void)
{
	std::lock_guard lock(sMutex);
	sTimings.clear();
}

SN_API const char *sn_debug_drain_log(void)
{
	std::deque<LogLine> lines;
	{
		std::lock_guard lock(sMutex);
		lines.swap(sLog);
	}
	std::string &out = tOutput;
	out = "[";
	for (size_t i = 0; i < lines.size(); ++i)
	{
		if (i > 0)
			out += ',';
		out += "{\"level\":";
		AppendNumber(out, lines[i].mLevel);
		out += ",\"message\":";
		AppendString(out, lines[i].mMessage.c_str());
		out += '}';
	}
	out += ']';
	return out.c_str();
}
