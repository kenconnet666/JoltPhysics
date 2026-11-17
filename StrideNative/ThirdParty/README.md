# Third-party sources

Copied once from upstream and maintained here; unlike Jolt they do not follow upstream. Only
permissive licenses (MIT, BSD, zlib, Apache-2.0); each library keeps its license file and source
headers. Upstream build files were removed: `StrideNative/CMakeLists.txt` builds everything.

When upstream ships a security or serious bug fix, compare against the commit below and merge by hand.

| Library | Version / commit | License | What is kept | Used by |
|---|---|---|---|---|
| [meshoptimizer](https://github.com/zeux/meshoptimizer) | v1.3, `9e1f07b159d3` | MIT | `src/` | runtime (decoders), tools (optimize, simplify, encode) |
| [basis_universal](https://github.com/BinomialLLC/basis_universal) | v2_50, `9bebe16726b3` | Apache-2.0 | `transcoder/`, `encoder/`, `zstd/`, `LICENSE`, `NOTICE`, `LICENSES/` | runtime (KTX2 transcoder), tools (encoder) |
| [recastnavigation](https://github.com/recastnavigation/recastnavigation) | main, `9f4ce64458df` (2026-02-27; last release v1.6.0 is from 2023) | zlib | `Recast`, `Detour`, `DetourCrowd`, `DetourTileCache` | runtime, tools |
| [ozz-animation](https://github.com/guillaumeblanc/ozz-animation) | 0.17.0, `744eb9d99f60` | MIT | `include/ozz/{base,animation/runtime,animation/offline}`, `src/{base,animation/runtime,animation/offline}` (no FBX/glTF tools) | runtime (sampling), tools (builders) |
| [ufbx](https://github.com/ufbx/ufbx) | v0.23.1, `26a482ae6687` | MIT or public domain | `ufbx.c`, `ufbx.h` | tools |
| [cgltf](https://github.com/jkuhlmann/cgltf) | v1.15, `360db1a95480` | MIT | `cgltf.h` | tools |
| [MikkTSpace](https://github.com/mmikk/MikkTSpace) | master, `3e895b49d05e` | zlib (in the headers) | `mikktspace.c/h` | tools |
| [xatlas](https://github.com/jpcy/xatlas) | master, `f700c7790aaa` | MIT | `source/xatlas/xatlas.cpp/h` | tools |
| [V-HACD](https://github.com/kmammou/v-hacd) | v4.1.0, `22ec20a7f8ea` | BSD-3-Clause | `include/VHACD.h` | tools |
| [msdfgen](https://github.com/Chlumsky/msdfgen) | v1.13.1, `ec84cd53590b` | MIT | `core/`, `msdfgen.h` (no `ext/`: fonts are read with stb_truetype) | tools |
| [stb](https://github.com/nothings/stb) | master, `9600522de539` | MIT or public domain | `stb_truetype.h`, `stb_image.h` | tools |

Local changes to these sources: none. If one becomes necessary, list it here with the reason.
