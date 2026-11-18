# StrideNative

The C++ side of stride-ts (a Stride-shaped TypeScript engine): Jolt plus the other native libraries,
built into two **multithreaded, SIMD** WebAssembly modules with a flat C API and a small JS layer.

| Module | Used by | Contents |
|---|---|---|
| `stride-native` | the game, in the browser (and Node for tests) | Jolt physics, meshopt decoding, KTX2/Basis transcoding, Recast/Detour navigation and crowds, ozz animation sampling |
| `stride-tools` | the editor's Node backend (asset pipeline) | glTF/FBX import, MikkTSpace tangents, meshopt optimize/simplify/encode, xatlas UV unwrap, V-HACD convex decomposition, Basis encoding, MSDF glyphs, ozz builders, navmesh baking |

There is no single-threaded build: pages must be cross-origin isolated
(`Cross-Origin-Opener-Policy: same-origin`, `Cross-Origin-Embedder-Policy: require-corp`).

## Layout

```
StrideNative/
  CMakeLists.txt        the only build file (Jolt comes in through ../Build/CMakeLists.txt, unchanged)
  Include/StrideNative/ C API: Core.h (init, memory, tasks), Runtime.h, Tools.h; Values.h (header-only value types shared with stride-ts columns: sn_decimal64, sn_date, sn_time, sn_datetime, sn_instant)
  Source/Core/          job system access, tasks, handle tables
  Source/Shared/        code both modules use (tiled navmesh builder and its file format)
  Source/Runtime/       stride-native exports
  Source/Tools/         stride-tools exports
  JS/                   native-module.mjs (shared), stride-native.mjs/.d.ts, stride-tools.mjs/.d.ts
  ThirdParty/           libraries copied once; versions and licenses in ThirdParty/README.md
  Tests/Native/         doctest executables, one per module (debug these in CLion)
  Tests/JS/             Node tests: tools produce assets, the runtime consumes them
  Tests/Browser/        COOP/COEP server and test page for a real browser
  Scripts/build-wasm.ps1
```

Each third-party library is a static library; our code is object libraries so every exported
function reaches the module; each wasm module is the final link of those.

## Threading

- Each module owns one Jolt `JobSystemThreadPool`. Physics steps and background work share it, so
  the cores are never oversubscribed. `sn_init(n)` starts `n` workers (`n >= 1`); the JS loader sizes
  Emscripten's Worker pool to the same number because Jolt creates its threads synchronously.
- **Tasks** (`RunTask`, `RunParallelTask`) run long work on the workers. Their status is an int32 in
  shared memory; the module calls `memory.atomic.notify` on it and JS awaits it with
  `Atomics.waitAsync`, so the main thread never blocks.
- **`ParallelFor`** is the synchronous form (animation sampling): the calling thread executes chunks
  while it waits, which is safe on the browser main thread; physics steps work the same way.
- What runs in parallel: meshopt buffers, KTX2 mip levels, navmesh tiles (runtime build and bake),
  animation instances, MSDF glyphs, and separate pipeline tasks (scene import, UV unwrap, convex
  decomposition, Basis encoding). Libraries that would start their own threads (xatlas, V-HACD, the
  Basis encoder) run single-threaded inside a task instead.
- Thread-safety rules followed: Basis needs one `ktx2_transcoder_state` per thread (and `clear()`
  after construction); Recast needs one `rcContext` per tile; Detour queries use one
  `dtNavMeshQuery` per querying thread; ozz needs one sampling context per instance.

## Build and test

Requirements: emsdk 6.0.8 at `C:\Users\Administrator\emsdk`, Node 22+; CMake and Ninja are taken
from CLion.

```powershell
pwsh StrideNative/Scripts/build-wasm.ps1          # both modules + Node tests -> Build/WASM_Release/dist
node StrideNative/Tests/Browser/serve.mjs 8190    # then open http://localhost:8190/
```

In CLion the CMake root is `StrideNative/`; profiles `Debug` (MinGW, runs
`StrideNativeRuntimeTests` / `StrideNativeToolsTests` with breakpoints) and `WASM-Release`
(Emscripten). Jolt's own unit tests still build from `Build/` (see stride-ts
`tools/build-jolt-wasm.ps1`).

`-DSTRIDE_NATIVE_BASIS_DEBUG=ON` prints Basis Universal's internal error messages.

## JS usage

```js
import { createStrideNative, TextureTarget } from './dist/stride-native.mjs';

const native = await createStrideNative();             // workers: hardwareConcurrency - 1
const texture = await native.transcodeKtx2(ktx2Bytes, TextureTarget.bc7);
const navMesh = native.loadNavMesh(bakedBytes);
const path = navMesh.findPath([0, 0, 0], [10, 0, 5]);
```

Inputs are copied into module memory and outputs copied back, except
`AnimationInstance.modelMatrices()`, which is a view of native memory for direct GPU upload.
