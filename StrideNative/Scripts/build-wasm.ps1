# 构建 npm 包 @stride-ts/native（StrideNative/JS）：编译 stride-native、stride-tools 两个 wasm 模块（多线程、SIMD），
# 用 tsc 编译 JS/src 的绑定，跑 Node 测试，最后用 pnpm pack 打成 JS/stride-ts-native-<版本>.tgz；
# 发布成 GitHub Release 用 Scripts/release-native.ps1，使用方（stride-ts）在 pnpm overrides 里引用它的下载地址。
#   pwsh StrideNative/Scripts/build-wasm.ps1 [-Config Release|Debug] [-SkipTests]
param(
    [ValidateSet('Debug', 'Release')]
    [string] $Config = 'Release',
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
$buildDir = Join-Path $root "Build\WASM_$Config"
$package = Join-Path $root 'JS'
$toolchain = Join-Path $Emsdk 'upstream\emscripten\cmake\Modules\Platform\Emscripten.cmake'

# wasm 模块编好后由 stride-native-package 目标放进 JS/dist
cmake -S $root -B $buildDir -G Ninja "-DCMAKE_BUILD_TYPE=$Config" "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
cmake --build $buildDir --target stride-native-package

pnpm --dir $package install --frozen-lockfile
pnpm --dir $package run build

if (-not $SkipTests) {
    node --test ((Join-Path $root 'Tests/JS/*.test.mjs') -replace '\\', '/')
}

pnpm --dir $package pack
