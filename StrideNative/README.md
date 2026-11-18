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
  JS/                   the npm package @stride-ts/native: src/ holds the TypeScript bindings
                        (native-module.ts, stride-native.ts, stride-tools.ts, *-core.d.mts for the
                        Emscripten factories); dist/ (generated) holds the compiled bindings next to
                        the wasm modules
  ThirdParty/           libraries copied once; versions and licenses in ThirdParty/README.md
  Tests/Native/         doctest executables, one per module (debug these in CLion)
  Tests/JS/             Node tests: tools produce assets, the runtime consumes them
  Tests/Browser/        COOP/COEP server and test page for a real browser
  Scripts/build-wasm.ps1     build, test and pack the npm package
  Scripts/release-native.ps1 publish the packed package as a GitHub Release
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

Requirements: emsdk 6.0.8 at `C:\Users\Administrator\emsdk`, Node 22+, pnpm 10; CMake and Ninja
are taken from CLion.

```powershell
pwsh StrideNative/Scripts/build-wasm.ps1          # wasm modules -> JS/dist, tsc, Node tests, pnpm pack
pwsh StrideNative/Scripts/build-wasm.ps1 -SourceDebug  # debug runtime at -O0 with DWARF, local use only
node StrideNative/Tests/Browser/serve.mjs 8190    # then open http://localhost:8190/
```

In CLion the CMake root is `StrideNative/`; profiles `Debug` (MinGW, runs
`StrideNativeRuntimeTests` / `StrideNativeToolsTests` with breakpoints) and `WASM-Release`
(Emscripten). Jolt's own unit tests still build from `Build/` (see stride-ts
`tools/build-jolt-wasm.ps1`).

`-DSTRIDE_NATIVE_BASIS_DEBUG=ON` prints Basis Universal's internal error messages.

The wasm modules keep their function names (`--profiling-funcs`, about 4% more after brotli), so
stack traces and Chrome profiles show C++ functions. `-DSTRIDE_NATIVE_DEBUG_CHECKS=ON` builds only the
runtime module, as `stride-native-core-debug`, with Jolt asserts (a failed assert logs to
`console.error` with the call stack) and Emscripten's `ASSERTIONS` and stack overflow checks;
`build-wasm.ps1` builds it into `Build/WASM_DebugChecks` next to the release modules, and
`createStrideNative({ debug: true })` loads it (a dynamic import, so a page downloads only one).
`-SourceDebug` compiles that module at `-O0` with DWARF for stepping through C++ in Chrome with the
C/C++ DevTools Support extension; such a package is large, do not release it.

## Package and release

`JS/` is the npm package `@stride-ts/native`. `build-wasm.ps1` puts the wasm modules into `JS/dist`
(the `stride-native-package` CMake target), compiles `JS/src` with tsc (relative imports are written
with `.ts` and rewritten to `.js`), runs the Node tests against `JS/dist` and packs
`JS/stride-ts-native-<version>.tgz`. The tests and the browser page load `JS/dist`, i.e. what ships.

Until the package is on npm it is released on GitHub: bump `version` in `JS/package.json`, run
`build-wasm.ps1`, commit and push to `origin/stride`, then `pwsh StrideNative/Scripts/release-native.ps1`.
It creates the release `native-v<version>` on kenconnet666/JoltPhysics with the tarball attached and
prints its download URL. Consumers install it with a pnpm override (stride-ts does this in
`pnpm-workspace.yaml`):

```yaml
overrides:
  '@stride-ts/native': https://github.com/kenconnet666/JoltPhysics/releases/download/native-v0.1.0/stride-ts-native-0.1.0.tgz
```

and depend on `"@stride-ts/native": "^0.1.0"` as usual. Bundlers must not pre-bundle it (Vite:
`optimizeDeps.exclude`), because the Emscripten modules find their `.wasm` and worker through
`import.meta.url`. The wasm files are also exported (`@stride-ts/native/stride-native-core.wasm`) for
setups that pass `locateFile`.

## JS usage

```ts
import { createStrideNative } from '@stride-ts/native';

const native = await createStrideNative();             // workers: hardwareConcurrency - 1
const texture = await native.transcodeKtx2(ktx2Bytes, 'bc7');
using navMesh = native.loadNavMesh(bakedBytes);        // or navMesh.destroy()
const path = navMesh.findPath([0, 0, 0], [10, 0, 5]);
```

Enumerations are string unions, handles are hidden, ids are branded (`BodyId`, `AgentId`), and failures are `NativeError` with a
string `code` (`'invalid-argument' | 'invalid-handle' | 'failed' | 'not-initialized'`).

Inputs are copied into module memory and outputs copied back, except
`AnimationInstance.modelMatrices()`, which is a view of native memory for direct GPU upload.
