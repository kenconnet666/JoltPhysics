// 示例插件：演示第三方插件怎么用插件 C API（Plugin.h），也是测试插件机制的对象。
//
// 它同时是"ABI 探针"：构建时放在运行时主模块的链接行上，主模块恰好导出它导入的符号。所以这里有意用到插件常用的
// C/C++ 标准库功能（new/delete、std::vector、std::string、std::mutex、std::unique_ptr、snprintf、数学函数），
// 第三方插件用这些就不会在加载时缺符号；要给插件开放新的库功能，就在这里用上它。
#include <StrideNative/Plugin.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

SN_PLUGIN(example)

// ---------------------------------------------------------------------------------------------
// 1. 零拷贝读写运行时的数据：让一组变换槽位上下起伏，按槽位分块并行
// ---------------------------------------------------------------------------------------------

namespace
{
	struct Wave
	{
		sn_transform_columns columns;
		const int32_t *slots;
		float time;
		float amplitude;
	};

	void ApplyWave(void *inUser, uint32_t inBegin, uint32_t inEnd)
	{
		const Wave &wave = *static_cast<const Wave *>(inUser);
		for (uint32_t i = inBegin; i < inEnd; ++i)
		{
			int32_t slot = wave.slots[i];
			wave.columns.position[slot * 3 + 1] = wave.amplitude * std::sin(wave.time + float(slot));
		}
	}
}

/// 把每个槽位的本地 y 设成 amplitude * sin(time + slot)。
SN_API int32_t sn_example_wave(int32_t transforms, const int32_t *slots, uint32_t count, float time, float amplitude)
{
	Wave wave { {}, slots, time, amplitude };
	int32_t status = sn_transforms_columns(transforms, &wave.columns);
	if (status != SN_OK)
		return status; // 运行时已经记下了错误
	for (uint32_t i = 0; i < count; ++i)
		if (slots[i] <= 0 || uint32_t(slots[i]) > wave.columns.capacity)
		{
			char message[96];
			std::snprintf(message, sizeof(message), "slot %d is outside 1..%u", slots[i], wave.columns.capacity);
			return sn_set_last_error(SN_ERROR_INVALID_ARGUMENT, message);
		}
	sn_parallel_for(count, ApplyWave, &wave);
	return SN_OK;
}

// ---------------------------------------------------------------------------------------------
// 2. 异步任务：统计字节的直方图，输出 256 个 uint32 和一段文字摘要
// ---------------------------------------------------------------------------------------------

namespace
{
	struct HistogramJob
	{
		std::vector<uint8_t> data;
	};

	int32_t RunHistogram(void *inUser, sn_task_context *inTask)
	{
		const HistogramJob &job = *static_cast<const HistogramJob *>(inUser);
		if (job.data.empty())
		{
			sn_task_fail(inTask, "sn_example_histogram: no data");
			return 1;
		}
		auto *counts = static_cast<uint32_t *>(sn_task_add_output(inTask, 256 * sizeof(uint32_t)));
		std::memset(counts, 0, 256 * sizeof(uint32_t));
		for (uint8_t byte : job.data)
			++counts[byte];

		uint32_t most = 0;
		for (uint32_t value = 1; value < 256; ++value)
			if (counts[value] > counts[most])
				most = value;
		std::string summary = std::to_string(job.data.size()) + " bytes, most common " + std::to_string(most);
		std::memcpy(sn_task_add_output(inTask, uint32_t(summary.size())), summary.data(), summary.size());
		sn_task_set_result(inTask, 0, int32_t(most));
		return 0;
	}
}

/// 复制 data，在工作线程上统计。结果 [0]：出现最多的字节；输出 [0]：256 个 uint32 计数，[1]：UTF-8 摘要。
SN_API int32_t sn_example_histogram(const void *data, uint32_t size)
{
	auto *job = new HistogramJob { std::vector<uint8_t>(static_cast<const uint8_t *>(data), static_cast<const uint8_t *>(data) + size) };
	return sn_run_task("ExampleHistogram", RunHistogram, job, [](void *inUser) { delete static_cast<HistogramJob *>(inUser); });
}

// ---------------------------------------------------------------------------------------------
// 3. 插件自己的对象和句柄：命名计数器。sn_shutdown 时全部释放
// ---------------------------------------------------------------------------------------------

namespace
{
	struct Counter
	{
		std::string name;
		int64_t total = 0;
	};

	std::mutex sCountersMutex;
	std::vector<std::unique_ptr<Counter>> sCounters;	// 句柄是下标加一，释放的位置留空
	bool sShutdownHooked = false;

	Counter *FindCounter(int32_t inHandle)
	{
		if (inHandle <= 0 || uint32_t(inHandle) > sCounters.size() || sCounters[inHandle - 1] == nullptr)
		{
			sn_set_last_error(SN_ERROR_INVALID_HANDLE, ("unknown counter " + std::to_string(inHandle)).c_str());
			return nullptr;
		}
		return sCounters[inHandle - 1].get();
	}
}

SN_API int32_t sn_example_counter_create(const char *name)
{
	std::lock_guard lock(sCountersMutex);
	if (!sShutdownHooked)
	{
		sShutdownHooked = true;
		sn_on_shutdown([](void *) { std::lock_guard lock(sCountersMutex); sCounters.clear(); }, nullptr);
	}
	sCounters.push_back(std::make_unique<Counter>(Counter { name != nullptr ? name : "", 0 }));
	return int32_t(sCounters.size());
}

/// 加上 value，返回新的总数（超出 int32 时截断）；句柄无效时返回 SN_ERROR_INVALID_HANDLE。
SN_API int32_t sn_example_counter_add(int32_t counter, int32_t value)
{
	std::lock_guard lock(sCountersMutex);
	Counter *c = FindCounter(counter);
	if (c == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	c->total += value;
	return int32_t(c->total);
}

/// 计数器的名字，在计数器释放前有效；句柄无效时返回空指针。
SN_API const char *sn_example_counter_name(int32_t counter)
{
	std::lock_guard lock(sCountersMutex);
	Counter *c = FindCounter(counter);
	return c != nullptr ? c->name.c_str() : nullptr;
}

SN_API void sn_example_counter_destroy(int32_t counter)
{
	std::lock_guard lock(sCountersMutex);
	if (FindCounter(counter) != nullptr)
		sCounters[counter - 1].reset();
}
