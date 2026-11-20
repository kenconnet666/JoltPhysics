# StrideNative: the C++ side of stride-ts, built as multithreaded SIMD wasm modules.
#   stride-native-core   运行时的主模块（变换、网格解码、动画、插件 API），游戏在浏览器里加载（测试时 Node 也加载）
#   stride-native-<功能>  运行时的侧模块（physics、texture、navigation），用到才加载，和主模块共用内存和任务系统
#   stride-tools-core    asset pipeline, loaded by the editor's Node backend
# Natively (MinGW/MSVC/Clang) the same code builds into static libraries plus a test executable,
# so everything can be debugged in an IDE before it is tried in wasm.
cmake_minimum_required(VERSION 3.20 FATAL_ERROR)
project(StrideNative VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(STRIDE_NATIVE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(THIRD_PARTY ${STRIDE_NATIVE_ROOT}/ThirdParty)
set(REPO_ROOT ${STRIDE_NATIVE_ROOT}/..)

# wasm 的初始和最大内存（工作线程池的大小由 JS 加载器经 Module.workerPoolSize 给）
set(STRIDE_NATIVE_INITIAL_MEMORY 67108864 CACHE STRING "Initial wasm memory in bytes")
set(STRIDE_NATIVE_MAXIMUM_MEMORY 2147483648 CACHE STRING "Maximum wasm memory in bytes")
if (EMSCRIPTEN)
	# Every object linked into a shared-memory module must be built with atomics, so -pthread has to
	# reach Jolt and all third-party code too. Set before add_subdirectory so Jolt inherits it.
	add_compile_options(-pthread -msimd128)
	add_link_options(-pthread)
	# 运行时拆成主模块加侧模块（Emscripten 动态链接）：侧模块的代码必须位置无关。主模块不可重定位，链接时
	# GOT 访问会放松成直接访问，实测步进速度没有差别
	add_compile_options(-fPIC)
endif()

# ---------------------------------------------------------------------------------------------
# Jolt, built through its own Build/CMakeLists.txt so its flags stay exactly as upstream sets them
# ---------------------------------------------------------------------------------------------
set(ENABLE_INSTALL OFF)
set(ENABLE_OBJECT_STREAM OFF)
set(JPH_USE_DX12 OFF)
set(JPH_USE_VK OFF)
set(JPH_USE_MTL OFF)
set(JPH_USE_CPU_COMPUTE OFF)
set(USE_WASM_SIMD ON)
set(INTERPROCEDURAL_OPTIMIZATION OFF)
# 调试版运行时（stride-native-core-debug）：打开 Jolt 的断言，链接时再加上 Emscripten 的运行时检查，见下面的 wasm 目标
option(STRIDE_NATIVE_DEBUG_CHECKS "Build only the runtime module, as stride-native-core-debug, with Jolt asserts and Emscripten runtime checks" OFF)
if (STRIDE_NATIVE_DEBUG_CHECKS)
	set(USE_ASSERTS ON)
endif()
add_subdirectory(${REPO_ROOT}/Build ${CMAKE_CURRENT_BINARY_DIR}/Jolt EXCLUDE_FROM_ALL)

# ---------------------------------------------------------------------------------------------
# Third-party libraries (copied once, see ThirdParty/README.md). Built without warnings: their
# code is not ours to fix, and Jolt's -Werror must not apply to them.
# ---------------------------------------------------------------------------------------------
function(stride_third_party_library NAME)
	cmake_parse_arguments(ARG "" "" "SOURCES;INCLUDES;DEFINES" ${ARGN})
	add_library(${NAME} STATIC ${ARG_SOURCES})
	# SYSTEM: their headers are included by our code, which builds with -Werror.
	target_include_directories(${NAME} SYSTEM PUBLIC ${ARG_INCLUDES})
	target_compile_definitions(${NAME} PUBLIC ${ARG_DEFINES})
	target_compile_options(${NAME} PRIVATE $<IF:$<CXX_COMPILER_ID:MSVC>,/W0,-w>)
	set_target_properties(${NAME} PROPERTIES FOLDER ThirdParty)
endfunction()

file(GLOB MESHOPT_SOURCES ${THIRD_PARTY}/meshoptimizer/src/*.cpp)
stride_third_party_library(sn_meshoptimizer SOURCES ${MESHOPT_SOURCES} INCLUDES ${THIRD_PARTY}/meshoptimizer/src)

file(GLOB RECAST_SOURCES
	${THIRD_PARTY}/recastnavigation/Recast/Source/*.cpp
	${THIRD_PARTY}/recastnavigation/Detour/Source/*.cpp
	${THIRD_PARTY}/recastnavigation/DetourCrowd/Source/*.cpp
	${THIRD_PARTY}/recastnavigation/DetourTileCache/Source/*.cpp)
stride_third_party_library(sn_recast SOURCES ${RECAST_SOURCES} INCLUDES
	${THIRD_PARTY}/recastnavigation/Recast/Include
	${THIRD_PARTY}/recastnavigation/Detour/Include
	${THIRD_PARTY}/recastnavigation/DetourCrowd/Include
	${THIRD_PARTY}/recastnavigation/DetourTileCache/Include)

file(GLOB_RECURSE OZZ_RUNTIME_SOURCES ${THIRD_PARTY}/ozz-animation/src/base/*.cc ${THIRD_PARTY}/ozz-animation/src/animation/runtime/*.cc)
stride_third_party_library(sn_ozz_runtime SOURCES ${OZZ_RUNTIME_SOURCES} INCLUDES ${THIRD_PARTY}/ozz-animation/include ${THIRD_PARTY}/ozz-animation/src)
file(GLOB OZZ_OFFLINE_SOURCES ${THIRD_PARTY}/ozz-animation/src/animation/offline/*.cc)
stride_third_party_library(sn_ozz_offline SOURCES ${OZZ_OFFLINE_SOURCES} INCLUDES ${THIRD_PARTY}/ozz-animation/include ${THIRD_PARTY}/ozz-animation/src)
target_link_libraries(sn_ozz_offline PUBLIC sn_ozz_runtime)
# ozz picks its SIMD types from the compiler's ISA macros. Our code sees Jolt's ISA flags, so ozz
# must be compiled with the same ones or the math types would differ between the two sides.
foreach (OZZ_TARGET sn_ozz_runtime sn_ozz_offline)
	target_compile_options(${OZZ_TARGET} PRIVATE $<TARGET_PROPERTY:Jolt,INTERFACE_COMPILE_OPTIONS>)
endforeach()

# Only the formats a WebGPU device can sample, plus RGBA as the universal fallback.
set(BASIS_TRANSCODER_DEFINES
	BASISD_SUPPORT_KTX2=1 BASISD_SUPPORT_KTX2_ZSTD=1
	BASISD_SUPPORT_PVRTC1=0 BASISD_SUPPORT_PVRTC2=0 BASISD_SUPPORT_ATC=0 BASISD_SUPPORT_FXT1=0)
option(STRIDE_NATIVE_BASIS_DEBUG "Print Basis Universal's internal error messages" OFF)
if (STRIDE_NATIVE_BASIS_DEBUG)
	list(APPEND BASIS_TRANSCODER_DEFINES BASISU_FORCE_DEVEL_MESSAGES=1)
endif()
stride_third_party_library(sn_basis_transcoder
	SOURCES ${THIRD_PARTY}/basis_universal/transcoder/basisu_transcoder.cpp
	INCLUDES ${THIRD_PARTY}/basis_universal/transcoder ${THIRD_PARTY}/basis_universal/zstd
	DEFINES ${BASIS_TRANSCODER_DEFINES})
# The runtime only decompresses zstd; the encoder needs the full library. Both define the same
# decoder symbols, so a module links exactly one of them.
stride_third_party_library(sn_zstd_decoder SOURCES ${THIRD_PARTY}/basis_universal/zstd/zstddeclib.c INCLUDES ${THIRD_PARTY}/basis_universal/zstd)
# 侧模块里的第三方库隐藏全部符号：侧模块只导出 C API，链接器能删掉没用到的代码
if (EMSCRIPTEN)
	foreach (SIDE_LIBRARY sn_recast sn_basis_transcoder sn_zstd_decoder)
		target_compile_options(${SIDE_LIBRARY} PRIVATE -fvisibility=hidden)
	endforeach()
endif()
stride_third_party_library(sn_zstd SOURCES ${THIRD_PARTY}/basis_universal/zstd/zstd.c INCLUDES ${THIRD_PARTY}/basis_universal/zstd)

# The encoder validates through the transcoder, so the tools get a transcoder with every format.
stride_third_party_library(sn_basis_transcoder_full
	SOURCES ${THIRD_PARTY}/basis_universal/transcoder/basisu_transcoder.cpp
	INCLUDES ${THIRD_PARTY}/basis_universal/transcoder ${THIRD_PARTY}/basis_universal/zstd
	DEFINES BASISD_SUPPORT_KTX2=1 BASISD_SUPPORT_KTX2_ZSTD=1)
file(GLOB BASIS_ENCODER_SOURCES ${THIRD_PARTY}/basis_universal/encoder/*.cpp ${THIRD_PARTY}/basis_universal/encoder/3rdparty/*.cpp)
list(FILTER BASIS_ENCODER_SOURCES EXCLUDE REGEX "basisu_wasm_")
stride_third_party_library(sn_basis_encoder
	SOURCES ${BASIS_ENCODER_SOURCES}
	INCLUDES ${THIRD_PARTY}/basis_universal/encoder
	DEFINES BASISU_SUPPORT_SSE=0 BASISU_SUPPORT_OPENCL=0)
target_link_libraries(sn_basis_encoder PUBLIC sn_basis_transcoder_full sn_zstd)

stride_third_party_library(sn_ufbx SOURCES ${THIRD_PARTY}/ufbx/ufbx.c INCLUDES ${THIRD_PARTY}/ufbx)
stride_third_party_library(sn_mikktspace SOURCES ${THIRD_PARTY}/MikkTSpace/mikktspace.c INCLUDES ${THIRD_PARTY}/MikkTSpace)
# Single-threaded: unwrap runs inside a task, the job system provides the parallelism.
stride_third_party_library(sn_xatlas SOURCES ${THIRD_PARTY}/xatlas/xatlas.cpp INCLUDES ${THIRD_PARTY}/xatlas DEFINES XA_MULTITHREADED=0)
file(GLOB MSDFGEN_SOURCES ${THIRD_PARTY}/msdfgen/core/*.cpp)
# Defining MSDFGEN_PUBLIC replaces the msdfgen-config.h that upstream generates at configure time.
stride_third_party_library(sn_msdfgen SOURCES ${MSDFGEN_SOURCES} INCLUDES ${THIRD_PARTY}/msdfgen DEFINES MSDFGEN_PUBLIC=)
# Header-only libraries get their implementation compiled in one of our own source files.
add_library(sn_header_only INTERFACE)
target_include_directories(sn_header_only SYSTEM INTERFACE ${THIRD_PARTY}/cgltf ${THIRD_PARTY}/stb ${THIRD_PARTY}/v-hacd)

# ---------------------------------------------------------------------------------------------
# Our code
# ---------------------------------------------------------------------------------------------
# OBJECT libraries: every exported C function must reach the final module even when nothing in C++
# references it, which a static archive would not guarantee.
function(stride_native_library NAME)
	add_library(${NAME} OBJECT ${ARGN})
	target_include_directories(${NAME} PUBLIC ${STRIDE_NATIVE_ROOT}/Include ${STRIDE_NATIVE_ROOT}/Source)
	if (MSVC)
		target_compile_options(${NAME} PRIVATE /W4 /WX)
	else()
		target_compile_options(${NAME} PRIVATE -Wall -Wextra -Werror -Wno-unused-parameter)
	endif()
endfunction()

file(GLOB CORE_SOURCES ${STRIDE_NATIVE_ROOT}/Source/Core/*.cpp)
stride_native_library(StrideNativeCore ${CORE_SOURCES})
target_link_libraries(StrideNativeCore PUBLIC Jolt)

file(GLOB RUNTIME_SOURCES ${STRIDE_NATIVE_ROOT}/Source/Runtime/*.cpp)
stride_native_library(StrideNativeRuntime ${RUNTIME_SOURCES})
target_link_libraries(StrideNativeRuntime PUBLIC StrideNativeCore sn_meshoptimizer sn_ozz_runtime)

# 侧模块（和工具模块共用）的代码：用 Jolt 和核心的头文件，但不链接它们，那些代码在主模块里。wasm 里隐藏
# 除 C API 以外的符号（SN_EXPORT 是 default 可见性）。
function(stride_native_feature_library NAME)
	stride_native_library(${NAME} ${ARGN})
	target_compile_options(${NAME} PRIVATE $<TARGET_PROPERTY:Jolt,INTERFACE_COMPILE_OPTIONS>)
	target_compile_definitions(${NAME} PRIVATE $<TARGET_PROPERTY:Jolt,INTERFACE_COMPILE_DEFINITIONS>)
	target_include_directories(${NAME} PRIVATE $<TARGET_PROPERTY:Jolt,INTERFACE_INCLUDE_DIRECTORIES>)
	if (EMSCRIPTEN)
		target_compile_options(${NAME} PRIVATE -fvisibility=hidden)
	endif()
endfunction()

# Code used by both the navigation module and the tools (navmesh building).
file(GLOB SHARED_SOURCES ${STRIDE_NATIVE_ROOT}/Source/Shared/*.cpp)
stride_native_feature_library(StrideNativeShared ${SHARED_SOURCES})
target_link_libraries(StrideNativeShared PUBLIC sn_recast)

stride_native_feature_library(StrideNativePhysics ${STRIDE_NATIVE_ROOT}/Source/Physics/PhysicsApi.cpp)
stride_native_feature_library(StrideNativeTexture ${STRIDE_NATIVE_ROOT}/Source/Texture/Ktx2Api.cpp)
target_link_libraries(StrideNativeTexture PUBLIC sn_basis_transcoder sn_zstd_decoder)
stride_native_feature_library(StrideNativeNavigation ${STRIDE_NATIVE_ROOT}/Source/Navigation/NavigationApi.cpp)
target_link_libraries(StrideNativeNavigation PUBLIC StrideNativeShared)

file(GLOB_RECURSE TOOLS_SOURCES ${STRIDE_NATIVE_ROOT}/Source/Tools/*.cpp)
stride_native_library(StrideNativeTools ${TOOLS_SOURCES})
target_link_libraries(StrideNativeTools PUBLIC StrideNativeCore StrideNativeShared sn_meshoptimizer sn_ozz_offline sn_basis_encoder
	sn_ufbx sn_mikktspace sn_xatlas sn_msdfgen sn_header_only)

# Objects of an OBJECT library only reach its direct consumers, so each module lists all of its own.
# 本机的运行时测试把所有功能链接进一个程序；wasm 的运行时只有核心，功能在各自的侧模块里。
set(RUNTIME_OBJECTS StrideNativeCore StrideNativeRuntime StrideNativeShared StrideNativePhysics StrideNativeTexture StrideNativeNavigation)
set(CORE_OBJECTS StrideNativeCore StrideNativeRuntime)
set(TOOLS_OBJECTS StrideNativeCore StrideNativeShared StrideNativeTools)

if (EMSCRIPTEN)
	set(WASM_COMMON_LINK_OPTIONS
		-sMODULARIZE=1 -sEXPORT_ES6=1
		-sALLOW_MEMORY_GROWTH=1
		-sINITIAL_MEMORY=${STRIDE_NATIVE_INITIAL_MEMORY} -sMAXIMUM_MEMORY=${STRIDE_NATIVE_MAXIMUM_MEMORY}
		-sSTACK_SIZE=1048576 -sDEFAULT_PTHREAD_STACK_SIZE=1048576
		# Jolt starts its workers synchronously, so the Worker pool must exist before main code runs.
		-sPTHREAD_POOL_SIZE=Module.workerPoolSize
		# wasmMemory: JS reads memory.buffer, which always spans the current (grown) shared memory.
		-sEXPORTED_RUNTIME_METHODS=wasmMemory,UTF8ToString
		-sFILESYSTEM=0 --no-entry
		# 留着函数名（name 段）：报错的调用栈和 Chrome 的性能分析里能看到 C++ 函数名，brotli 后约多 4%
		--profiling-funcs)

	if (STRIDE_NATIVE_DEBUG_CHECKS)
		set(FLAVOR -debug)
		list(APPEND WASM_COMMON_LINK_OPTIONS -sASSERTIONS=1 -sSTACK_OVERFLOW_CHECK=2)
	else()
		set(FLAVOR "")
	endif()

	# 侧模块：功能的代码加上它独有的第三方库，只导出 C API。
	function(stride_native_side_module NAME)
		set(TARGET stride-native-${NAME}${FLAVOR})
		add_executable(${TARGET})
		target_link_libraries(${TARGET} PRIVATE ${ARGN})
		target_link_options(${TARGET} PRIVATE -sSIDE_MODULE=2 --profiling-funcs)
		set_target_properties(${TARGET} PROPERTIES SUFFIX ".wasm")
	endfunction()

	# 物理：Jolt 除 Core 以外的部分。Jolt/Core（任务系统、分配器、Trace……）只在主模块里，侧模块里不能再有一份，
	# 否则那些全局状态会有两份。Jolt 的源文件按原样的编译选项再编一遍（位置无关、隐藏符号）。
	get_target_property(JOLT_SOURCES Jolt SOURCES)
	list(FILTER JOLT_SOURCES INCLUDE REGEX "\\.cpp$")
	list(FILTER JOLT_SOURCES EXCLUDE REGEX "/Jolt/Core/")
	add_library(sn_jolt_physics STATIC ${JOLT_SOURCES})
	target_compile_options(sn_jolt_physics PRIVATE $<TARGET_PROPERTY:Jolt,COMPILE_OPTIONS> -fvisibility=hidden)
	target_compile_definitions(sn_jolt_physics PRIVATE $<TARGET_PROPERTY:Jolt,COMPILE_DEFINITIONS>)
	target_include_directories(sn_jolt_physics PRIVATE $<TARGET_PROPERTY:Jolt,INCLUDE_DIRECTORIES>)
	stride_native_side_module(physics StrideNativePhysics sn_jolt_physics)
	stride_native_side_module(texture StrideNativeTexture)
	# Objects of an OBJECT library only reach its direct consumers: list Shared as well.
	stride_native_side_module(navigation StrideNativeNavigation StrideNativeShared)
	set(SIDE_MODULES stride-native-physics${FLAVOR} stride-native-texture${FLAVOR} stride-native-navigation${FLAVOR})

	# 示例插件，同时是插件 ABI 的探针：它放在主模块的链接行上，主模块就会导出它导入的全部符号，第三方插件
	# 能用的就是这些（见 Include/StrideNative/Plugin.h）。不进 npm 包，测试从构建目录加载它。
	include(${STRIDE_NATIVE_ROOT}/CMake/StrideNativePlugin.cmake)
	stride_native_plugin(example-plugin ${STRIDE_NATIVE_ROOT}/Plugins/Example/ExamplePlugin.cpp)

	# 核心的 C API（Core.h、Debug.h、Runtime.h、Plugin.h 里的 SN_API）全部列进 EXPORTED_FUNCTIONS。emcc 把侧模块导入的符号记成
	# "已请求的导出"，不再按 EMSCRIPTEN_KEEPALIVE 放到 Module 上（emscripten.py 的 should_export 只认 EXPORTED_FUNCTIONS），
	# 不列出来的话，被插件或侧模块用到的 sn_alloc、sn_transforms_columns 之类在 JS 里就没有了
	set(CORE_API_EXPORTS)
	foreach (HEADER Core.h Debug.h Runtime.h Plugin.h)
		file(STRINGS ${STRIDE_NATIVE_ROOT}/Include/StrideNative/${HEADER} API_LINES REGEX "^SN_API ")
		foreach (LINE IN LISTS API_LINES)
			if (LINE MATCHES "^SN_API [^(]*[ *]([A-Za-z0-9_]+)\\(")
				list(APPEND CORE_API_EXPORTS _${CMAKE_MATCH_1})
			endif()
		endforeach()
	endforeach()
	list(JOIN CORE_API_EXPORTS "," CORE_API_EXPORTS)

	set(RUNTIME_MODULE stride-native-core${FLAVOR})
	add_executable(${RUNTIME_MODULE})
	target_link_libraries(${RUNTIME_MODULE} PRIVATE ${CORE_OBJECTS})
	# 主模块（MAIN_MODULE=2）只导出链接行上侧模块导入的符号；不自动加载它们，JS 用 dynamicLibraries 指定要哪些；
	# LDSO 让 JS 拿到侧模块的导出
	target_link_options(${RUNTIME_MODULE} PRIVATE ${WASM_COMMON_LINK_OPTIONS}
		-sEXPORT_NAME=createStrideNativeCore -sENVIRONMENT=web,worker,node
		-sMAIN_MODULE=2 -sAUTOLOAD_DYLIBS=0 -sEXPORTED_RUNTIME_METHODS=wasmMemory,UTF8ToString,LDSO
		-sEXPORTED_FUNCTIONS=${CORE_API_EXPORTS})
	# 侧模块放在库的位置（目标文件之后）：放在前面时，主模块自己有弱定义的符号（typeinfo 之类）会被解析成从侧模块导入
	foreach (SIDE_MODULE ${SIDE_MODULES} example-plugin)
		target_link_libraries(${RUNTIME_MODULE} PRIVATE $<TARGET_FILE:${SIDE_MODULE}>)
		add_dependencies(${RUNTIME_MODULE} ${SIDE_MODULE})
	endforeach()
	set_target_properties(${RUNTIME_MODULE} PROPERTIES SUFFIX ".mjs")
	set(PACKAGE_FILES $<TARGET_FILE:${RUNTIME_MODULE}> $<TARGET_FILE_DIR:${RUNTIME_MODULE}>/${RUNTIME_MODULE}.wasm)
	set(PACKAGE_TARGETS ${RUNTIME_MODULE} ${SIDE_MODULES})
	foreach (SIDE_MODULE ${SIDE_MODULES})
		list(APPEND PACKAGE_FILES $<TARGET_FILE:${SIDE_MODULE}>)
	endforeach()

	# 工具模块只在 Node 里跑资源管线，没有调试版
	if (NOT STRIDE_NATIVE_DEBUG_CHECKS)
		add_executable(stride-tools-core)
		target_link_libraries(stride-tools-core PRIVATE ${TOOLS_OBJECTS})
		target_link_options(stride-tools-core PRIVATE ${WASM_COMMON_LINK_OPTIONS}
			-sEXPORT_NAME=createStrideToolsCore -sENVIRONMENT=node,worker)
		set_target_properties(stride-tools-core PROPERTIES SUFFIX ".mjs")
		list(APPEND PACKAGE_FILES $<TARGET_FILE:stride-tools-core> $<TARGET_FILE_DIR:stride-tools-core>/stride-tools-core.wasm)
		list(APPEND PACKAGE_TARGETS stride-tools-core)
	endif()

	# npm 包 @stride-ts/native（JS/）的 dist/ 放 Emscripten 的产出，旁边是 tsc 从 JS/src 编译出的绑定
	# （JS/ 里 pnpm run build）。正式版和调试版的文件名不同，两次构建都写进同一个目录。
	set(STRIDE_NATIVE_PACKAGE_DIST ${STRIDE_NATIVE_ROOT}/JS/dist CACHE PATH "Where the wasm modules of the @stride-ts/native package go")
	set(PLUGIN_SDK_HEADERS ${STRIDE_NATIVE_ROOT}/Include/StrideNative/Core.h ${STRIDE_NATIVE_ROOT}/Include/StrideNative/Debug.h ${STRIDE_NATIVE_ROOT}/Include/StrideNative/Runtime.h ${STRIDE_NATIVE_ROOT}/Include/StrideNative/Plugin.h)
	# 每次构建都跑（只拷有变化的文件）：dist/ 被清掉以后，没有重新链接也能补回来
	add_custom_target(stride-native-package ALL
		COMMAND ${CMAKE_COMMAND} -E make_directory ${STRIDE_NATIVE_PACKAGE_DIST}
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PACKAGE_FILES} ${STRIDE_NATIVE_PACKAGE_DIST}
		# 插件 SDK：第三方插件只需要这几个头文件和 CMake 函数（见 Plugins/README.md）
		COMMAND ${CMAKE_COMMAND} -E make_directory ${STRIDE_NATIVE_PACKAGE_DIST}/sdk/Include/StrideNative ${STRIDE_NATIVE_PACKAGE_DIST}/sdk/CMake
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PLUGIN_SDK_HEADERS} ${STRIDE_NATIVE_PACKAGE_DIST}/sdk/Include/StrideNative
		COMMAND ${CMAKE_COMMAND} -E copy_if_different ${STRIDE_NATIVE_ROOT}/CMake/StrideNativePlugin.cmake ${STRIDE_NATIVE_PACKAGE_DIST}/sdk/CMake
		DEPENDS ${PACKAGE_TARGETS}
		VERBATIM)
else()
	# Native test executables, one per module (the two link different zstd/transcoder builds):
	# the same C API as the wasm modules, debuggable in the IDE.
	foreach (MODULE Runtime Tools)
		file(GLOB TEST_SOURCES ${STRIDE_NATIVE_ROOT}/Tests/Native/${MODULE}/*.cpp)
		add_executable(StrideNative${MODULE}Tests ${STRIDE_NATIVE_ROOT}/Tests/Native/TestMain.cpp ${TEST_SOURCES})
		target_include_directories(StrideNative${MODULE}Tests PRIVATE ${REPO_ROOT}/UnitTests ${STRIDE_NATIVE_ROOT}/Tests/Native)
		string(TOUPPER ${MODULE} MODULE_UPPER)
		target_link_libraries(StrideNative${MODULE}Tests PRIVATE ${${MODULE_UPPER}_OBJECTS})
		target_compile_definitions(StrideNative${MODULE}Tests PRIVATE STRIDE_NATIVE_TEST_DATA="${STRIDE_NATIVE_ROOT}/Tests/Data")
	endforeach()
endif()
