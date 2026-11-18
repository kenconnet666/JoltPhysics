# stride_native_plugin(<目标> <源文件>...)：把源文件编成 StrideNative 的 wasm 插件 <目标>.wasm（Emscripten 侧模块）。
# 用法：include() 这个文件，用和运行时相同版本的 emsdk 配置 CMake（emcmake cmake ...），见 Plugins/README.md。
# 头文件在这个文件旁边的 ../Include（仓库里是 StrideNative/Include，npm 包里是 sdk/Include）。
set(STRIDE_NATIVE_PLUGIN_INCLUDE ${CMAKE_CURRENT_LIST_DIR}/../Include)

function(stride_native_plugin TARGET)
	if (NOT EMSCRIPTEN)
		message(FATAL_ERROR "stride_native_plugin(${TARGET}): configure with the Emscripten toolchain (emcmake cmake ...)")
	endif()
	add_executable(${TARGET} ${ARGN})
	target_include_directories(${TARGET} PRIVATE ${STRIDE_NATIVE_PLUGIN_INCLUDE})
	# 和运行时一致：共享内存（-pthread）、SIMD；侧模块的代码位置无关，只导出 SN_API 标的函数
	target_compile_options(${TARGET} PRIVATE -pthread -msimd128 -fPIC -fvisibility=hidden)
	target_link_options(${TARGET} PRIVATE -pthread -sSIDE_MODULE=2 --profiling-funcs)
	set_target_properties(${TARGET} PROPERTIES SUFFIX ".wasm")
endfunction()
