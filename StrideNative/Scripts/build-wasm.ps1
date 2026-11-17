# Builds the stride-native and stride-tools wasm modules (multithreaded, SIMD) and runs the JS tests.
#   pwsh StrideNative/Scripts/build-wasm.ps1 [-Config Release|Debug] [-SkipTests]
# Output: StrideNative/Build/WASM_<Config>/dist (Emscripten modules plus the JS wrappers).
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
$toolchain = Join-Path $Emsdk 'upstream\emscripten\cmake\Modules\Platform\Emscripten.cmake'

cmake -S $root -B $buildDir -G Ninja "-DCMAKE_BUILD_TYPE=$Config" "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
cmake --build $buildDir --target stride-native-core stride-tools-core

if (-not $SkipTests) {
    $env:STRIDE_NATIVE_DIST = Join-Path $buildDir 'dist'
    node --test ((Join-Path $root 'Tests/JS/*.test.mjs') -replace '\\', '/')
}
