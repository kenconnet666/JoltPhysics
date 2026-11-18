// 插件 C API：第三方 wasm 插件（和运行时模块一起加载的 Emscripten 侧模块）能调用的东西。插件和内置的侧模块
// （物理、贴图、导航）一样，跟运行时共用内存、分配器和任务系统，所以同样能多线程、零拷贝地读写运行时的数据。
//
// 插件的规矩（详见 Plugins/README.md）：
// - 用和运行时相同的 emsdk，通过 CMake 函数 stride_native_plugin() 编译；导出的函数用 SN_API。
// - 只调用这个头文件、Core.h、Runtime.h 的 C API，以及示例插件用到的 C/C++ 标准库功能：运行时模块恰好导出
//   示例插件（"ABI 探针"）导入的那些符号，别的符号在加载时报 "undefined symbol"。
// - 插件只在启动时加载（createStrideNative({ features: [...] })），每个线程都加载一份。
// - 不要依赖 StrideNative 的 C++ 内部实现：句柄表之类的模板静态变量每个模块各有一份。
#pragma once

#include <StrideNative/Core.h>
#include <StrideNative/Runtime.h>

/// 插件 API 有不兼容的改动时加一。
#define SN_PLUGIN_ABI_VERSION 1

/// 正在运行的核心的插件 ABI 版本。
SN_API uint32_t sn_plugin_abi_version(void);

/// 每个插件（内置的侧模块也一样）在一个源文件里写一次 SN_PLUGIN(名字)：JS 加载时调用 sn_plugin_abi_<名字>，
/// 和核心的版本不一致就拒绝加载。名字就是 JS 里 NativeFeature 的 name。
#define SN_PLUGIN(name) \
	SN_API uint32_t sn_plugin_abi_##name(void) { return SN_PLUGIN_ABI_VERSION; }

/// 记下当前线程的错误消息（JS 通过 sn_last_error 读），返回 code。
SN_API int32_t sn_set_last_error(int32_t code, const char *message);

/// 把 [0, count) 分块放到任务系统上执行 fn(user, begin, end)，等全部完成才返回；等待时当前线程也执行分块，
/// 所以在浏览器主线程上调用也安全。
typedef void (*sn_parallel_fn)(void *user, uint32_t begin, uint32_t end);
SN_API void sn_parallel_for(uint32_t count, sn_parallel_fn fn, void *user);

/// 异步任务，JS 照内置任务那样等待（NativeModule.awaitTask：sn_task_status、sn_task_result、sn_task_output_*）。
/// fn 在工作线程上运行，返回 0 表示成功；失败时返回非 0，或者先调用 sn_task_fail 说明原因。
typedef struct sn_task_context sn_task_context;
typedef int32_t (*sn_task_fn)(void *user, sn_task_context *task);
/// 启动任务，返回任务 id（> 0）。name 要一直有效（用字符串字面量），性能分析里显示它。free_user 不为空时，
/// fn 返回后用 user 调用它。
SN_API int32_t sn_run_task(const char *name, sn_task_fn fn, void *user, void (*free_user)(void *user));
/// 小整数结果，index 是 0..SN_TASK_RESULT_COUNT-1（8 个），超出的忽略。
SN_API void sn_task_set_result(sn_task_context *task, uint32_t index, int32_t value);
/// 加一个归任务所有的输出缓冲（sn_task_release 时释放），返回它的地址，由 fn 填写。
SN_API void *sn_task_add_output(sn_task_context *task, uint32_t size);
/// 标记任务失败；第一条消息有效。
SN_API void sn_task_fail(sn_task_context *task, const char *message);

/// sn_shutdown 时调用 fn(user)（后注册的先调用），插件在这里释放自己的对象。注册一直有效，跨多次 init/shutdown，
/// 所以只注册一次（比如第一次用到时）。
SN_API void sn_on_shutdown(void (*fn)(void *user), void *user);
