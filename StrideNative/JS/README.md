# @stride-ts/native

StrideNative compiled to multithreaded SIMD WebAssembly, with TypeScript bindings: Jolt physics shaped like
Stride.BepuPhysics (compound and mesh colliders, Stride's character controller, 32 collision layers, ray and
sphere casts, interpolation) with a shared transform store, Recast/Detour navigation, ozz animation sampling, meshopt and KTX2 decoding
(`@stride-ts/native`, browser and Node), and the asset tools used by the pipeline (`@stride-ts/native/tools`,
Node: scene import, tangents, mesh optimization, UV unwrap, convex decomposition, Basis encoding, MSDF
fonts, navmesh baking).

```ts
import { createStrideNative } from '@stride-ts/native';

const native = await createStrideNative();             // workers: hardwareConcurrency - 1, at most 8
using transforms = native.createTransforms(1024);
using world = native.createPhysicsWorld(1024, { transforms });
```

- The memory is a `SharedArrayBuffer`: pages must be cross-origin isolated
  (`Cross-Origin-Opener-Policy: same-origin`, `Cross-Origin-Embedder-Policy: require-corp`).
- The Emscripten modules locate their `.wasm` and worker through `import.meta.url`; exclude this package
  from bundler pre-bundling (Vite: `optimizeDeps: { exclude: ['@stride-ts/native'] }`). The wasm files
  are exported as `@stride-ts/native/stride-native-core.wasm`, `@stride-ts/native/stride-native-core-debug.wasm`
  and `@stride-ts/native/stride-tools-core.wasm`.
- `createStrideNative({ debug: true })` loads the debug runtime: Jolt asserts and Emscripten runtime checks,
  slower than the release one; use it during development.
- Source, build and release notes: `StrideNative/README.md` in
  [kenconnet666/JoltPhysics](https://github.com/kenconnet666/JoltPhysics/tree/stride/StrideNative).

MIT license.
