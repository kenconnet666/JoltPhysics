# 构建 npm 包 @stride-ts/native（StrideNative/JS）：编译 stride-native、stride-tools 两个 wasm 模块（多线程、SIMD），
# 再编一个调试版的运行时模块 stride-native-core-debug（Jolt 断言加 Emscripten 运行时检查），用 tsc 编译 JS/src 的绑定，
# 跑 Node 测试，最后用 pnpm pack 打成 JS/stride-ts-native-<版本>.tgz；发布成 GitHub Release 用 Scripts/release-native.ps1，
# 使用方（stride-ts）在 pnpm overrides 里引用它的下载地址。
#   pwsh StrideNative/Scripts/build-wasm.ps1 [-SourceDebug] [-SkipTests]
# -SourceDebug：调试版改成 -O0 加 DWARF，在 Chrome（C/C++ DevTools Support 扩展）里按 C++ 源码单步。只在本机用，
# 这样打出来的包很大，不要发布。
param(
    [switch] $SourceDebug,
    [switch] $SkipTests,
    [string] $Emsdk = 'C:\Users\Administrator\emsdk'
)

$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $true

. (Join-Path $Emsdk 'emsdk_env.ps1') *> $null
# CMake and Ninja are not installed system-wide; use the copies bundled with CLion.
$clionBin = Join-Path $env:LOCALAPPDATA 'Programs\CLion\bin'
$env:PATH = "$clionBin\cmake\win\x64\bin;$clionBin\ninja\win\x64;$env:PATH"

$root = Split-Path $PSScriptRoot -Parent
$package = Join-Path $root 'JS'
$toolchain = Join-Path $Emsdk 'upstream\emscripten\cmake\Modules\Platform\Emscripten.cmake'

# wasm 模块编好后由 stride-native-package 目标放进 JS/dist：先是正式版的两个模块，再是调试版的运行时模块
$releaseDir = Join-Path $root 'Build\WASM_Release'
cmake -S $root -B $releaseDir -G Ninja -DCMAKE_BUILD_TYPE=Release "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
cmake --build $releaseDir --target stride-native-package

$debugDir = Join-Path $root 'Build\WASM_DebugChecks'
$debugType = $SourceDebug ? 'Debug' : 'Release'
cmake -S $root -B $debugDir -G Ninja "-DCMAKE_BUILD_TYPE=$debugType" -DSTRIDE_NATIVE_DEBUG_CHECKS=ON "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
cmake --build $debugDir --target stride-native-package

pnpm --dir $package install --frozen-lockfile
pnpm --dir $package run build

if (-not $SkipTests) {
    node --test ((Join-Path $root 'Tests/JS/*.test.mjs') -replace '\\', '/')
}

pnpm --dir $package pack
