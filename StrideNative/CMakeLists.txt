# StrideNative: the C++ side of stride-ts, built as two multithreaded SIMD wasm modules.
#   stride-native  runtime, loaded by the game in the browser (and by Node for tests)
#   stride-tools   asset pipeline, loaded by the editor's Node backend
# Natively (MinGW/MSVC/Clang) the same code builds into static libraries plus a test executable,
# so everything can be debugged in an IDE before it is tried in wasm.
cmake_minimum_required(VERSION 3.20 FATAL_ERROR)
project(StrideNative VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(STRIDE_NATIVE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(THIRD_PARTY ${STRIDE_NATIVE_ROOT}/ThirdParty)
set(REPO_ROOT ${STRIDE_NATIVE_ROOT}/..)

# Initial worker pool for wasm; the JS loader passes the real value through Module.workerPoolSize.
set(STRIDE_NATIVE_INITIAL_MEMORY 67108864 CACHE STRING "Initial wasm memory in bytes")
set(STRIDE_NATIVE_MAXIMUM_MEMORY 2147483648 CACHE STRING "Maximum wasm memory in bytes")

if (EMSCRIPTEN)
	# Every object linked into a shared-memory module must be built with atomics, so -pthread has to
	# reach Jolt and all third-party code too. Set before add_subdirectory so Jolt inherits it.
	add_compile_options(-pthread -msimd128)
	add_link_options(-pthread)
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
stride_third_party_library(sn_basis_transcoder
	SOURCES ${THIRD_PARTY}/basis_universal/transcoder/basisu_transcoder.cpp
	INCLUDES ${THIRD_PARTY}/basis_universal/transcoder ${THIRD_PARTY}/basis_universal/zstd
	DEFINES ${BASIS_TRANSCODER_DEFINES})
# The runtime only decompresses zstd; the encoder needs the full library. Both define the same
# decoder symbols, so a module links exactly one of them.
stride_third_party_library(sn_zstd_decoder SOURCES ${THIRD_PARTY}/basis_universal/zstd/zstddeclib.c INCLUDES ${THIRD_PARTY}/basis_universal/zstd)
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

# Code used by both modules (navmesh building).
file(GLOB SHARED_SOURCES ${STRIDE_NATIVE_ROOT}/Source/Shared/*.cpp)
stride_native_library(StrideNativeShared ${SHARED_SOURCES})
target_link_libraries(StrideNativeShared PUBLIC StrideNativeCore sn_recast)

file(GLOB_RECURSE RUNTIME_SOURCES ${STRIDE_NATIVE_ROOT}/Source/Runtime/*.cpp)
stride_native_library(StrideNativeRuntime ${RUNTIME_SOURCES})
target_link_libraries(StrideNativeRuntime PUBLIC StrideNativeShared sn_meshoptimizer sn_ozz_runtime sn_basis_transcoder sn_zstd_decoder)

file(GLOB_RECURSE TOOLS_SOURCES ${STRIDE_NATIVE_ROOT}/Source/Tools/*.cpp)
stride_native_library(StrideNativeTools ${TOOLS_SOURCES})
target_link_libraries(StrideNativeTools PUBLIC StrideNativeShared sn_meshoptimizer sn_ozz_offline sn_basis_encoder
	sn_ufbx sn_mikktspace sn_xatlas sn_msdfgen sn_header_only)

# Objects of an OBJECT library only reach its direct consumers, so each module lists all of its own.
set(RUNTIME_OBJECTS StrideNativeCore StrideNativeShared StrideNativeRuntime)
set(TOOLS_OBJECTS StrideNativeCore StrideNativeShared StrideNativeTools)

if (EMSCRIPTEN)
	set(WASM_COMMON_LINK_OPTIONS
		-sMODULARIZE=1 -sEXPORT_ES6=1
		-sALLOW_MEMORY_GROWTH=1
		-sINITIAL_MEMORY=${STRIDE_NATIVE_INITIAL_MEMORY} -sMAXIMUM_MEMORY=${STRIDE_NATIVE_MAXIMUM_MEMORY}
		-sSTACK_SIZE=1048576 -sDEFAULT_PTHREAD_STACK_SIZE=1048576
		# Jolt starts its workers synchronously, so the Worker pool must exist before main code runs.
		-sPTHREAD_POOL_SIZE=Module.workerPoolSize
		-sEXPORTED_RUNTIME_METHODS=HEAPU8,HEAP32,HEAPU32,HEAPF32,UTF8ToString
		-sFILESYSTEM=0 --no-entry)

	add_executable(stride-native-core)
	target_link_libraries(stride-native-core PRIVATE ${RUNTIME_OBJECTS})
	target_link_options(stride-native-core PRIVATE ${WASM_COMMON_LINK_OPTIONS}
		-sEXPORT_NAME=createStrideNativeCore -sENVIRONMENT=web,worker,node)
	set_target_properties(stride-native-core PROPERTIES SUFFIX ".mjs")

	add_executable(stride-tools-core)
	target_link_libraries(stride-tools-core PRIVATE ${TOOLS_OBJECTS})
	target_link_options(stride-tools-core PRIVATE ${WASM_COMMON_LINK_OPTIONS}
		-sEXPORT_NAME=createStrideToolsCore -sENVIRONMENT=node,worker)
	set_target_properties(stride-tools-core PROPERTIES SUFFIX ".mjs")

	# dist/ holds what JS consumers load: the Emscripten output next to our hand-written wrappers.
	set(DIST_DIR ${CMAKE_BINARY_DIR}/dist)
	foreach (TARGET_NAME stride-native-core stride-tools-core)
		add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E make_directory ${DIST_DIR}
			COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:${TARGET_NAME}> $<TARGET_FILE_DIR:${TARGET_NAME}>/${TARGET_NAME}.wasm ${DIST_DIR}
			COMMAND ${CMAKE_COMMAND} -E copy_directory ${STRIDE_NATIVE_ROOT}/JS ${DIST_DIR}
			VERBATIM)
	endforeach()
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
