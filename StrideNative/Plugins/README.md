# StrideNative 插件

插件是自己写的 C/C++ 代码编成的 wasm 侧模块，和运行时的核心模块一起加载，跟内置的物理、贴图、导航模块地位一样：
共用同一块共享内存、同一个分配器和任务系统，能零拷贝地读写运行时的数据（比如变换存储的列），能把工作分到所有线程上。

[Example/ExamplePlugin.cpp](Example/ExamplePlugin.cpp) 是完整的例子：并行改写变换槽位、异步任务、插件自己的对象和关机清理；
`Tests/JS/stride-native.test.mjs` 里有它的 JS 用法。

## 能做什么、不能做什么

- **只用 C API**：[Plugin.h](../Include/StrideNative/Plugin.h)（任务、并行、错误、关机钩子）、Core.h（内存、任务结果）、
  Runtime.h（变换存储）。不要用 StrideNative 的 C++ 内部实现（`Core/*.h`）：句柄表之类的模板静态变量每个模块各有一份。
- **标准库只用示例插件用到的那些**。核心模块恰好导出示例插件（"ABI 探针"，它在核心的链接行上）导入的符号，插件用了别的
  库函数，加载时报 `undefined symbol`。需要新的库功能时，在示例插件里用上它、重编运行时，下一个版本就有了。
- **和运行时用同一个 emsdk 版本**（README 的"Build and test"里写着），C++ 标准库的 ABI 要一致。
- **只在启动时加载**，每个线程一份；插件导出的函数在哪个线程都能调用，并发安全由插件自己负责。
- 调试版运行时（`debug: true`）也能加载只用 C API 的插件；内置模块各有调试版，是因为它们用 Jolt 的内部结构。

## 构建

npm 包的 `dist/sdk/` 里有头文件和 CMake 函数（仓库里是 `StrideNative/Include` 和 `StrideNative/CMake`）：

```cmake
cmake_minimum_required(VERSION 3.20)
project(MyPlugin CXX)
set(CMAKE_CXX_STANDARD 17)
include(node_modules/@stride-ts/native/dist/sdk/CMake/StrideNativePlugin.cmake)
stride_native_plugin(my-plugin MyPlugin.cpp)   # 输出 my-plugin.wasm
```

```powershell
emcmake cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

源码里写一次 `SN_PLUGIN(my_plugin)`（名字是 C 标识符，和 JS 里的 `name` 一致），导出的函数用 `SN_API`：

```cpp
#include <StrideNative/Plugin.h>

SN_PLUGIN(my_plugin)

SN_API int32_t my_plugin_lift(int32_t transforms, const int32_t *slots, uint32_t count, float height)
{
	sn_transform_columns columns;
	if (int32_t status = sn_transforms_columns(transforms, &columns); status != SN_OK)
		return status;
	for (uint32_t i = 0; i < count; ++i)
		if (slots[i] <= 0 || uint32_t(slots[i]) > columns.capacity)
			return sn_set_last_error(SN_ERROR_INVALID_ARGUMENT, "slot out of range");
	for (uint32_t i = 0; i < count; ++i)
		columns.position[slots[i] * 3 + 1] += height;
	return SN_OK;
}
```

## 在 JS 里用

```ts
import { createStrideNative, type NativeFeature } from '@stride-ts/native';

// 类型参数列出导出的函数名，native.plugin() 就按它给出带类型的函数表
const myPlugin: NativeFeature<'my_plugin_lift'> = {
	name: 'my_plugin',
	url: new URL('./my-plugin.wasm', import.meta.url),
};

const native = await createStrideNative({ features: [myPlugin] });
const { my_plugin_lift } = native.plugin(myPlugin);
const transforms = native.createTransforms(1024);
const slots = transforms.allocate(10);
native.withArrays([Int32Array.from(slots)], (pointer) =>
	native.check(my_plugin_lift(native.handleOf(transforms), pointer, slots.length, 2), 'my_plugin_lift'));
```

- 指针参数用 `alloc` / `free`、`withArrays`、`withScratch` 准备；返回负数表示失败，`check` 把它变成带 `sn_last_error`
  消息的 `NativeError`。
- 异步任务（`sn_run_task`）返回任务 id，用 `native.awaitTask(id, what, (task) => ...)` 等它，结果和输出缓冲照内置任务那样读。
- 加载失败的原因会写在 `createStrideNative` 抛出的 `NativeError` 里：缺符号、没写 `SN_PLUGIN`、ABI 版本不一致。
