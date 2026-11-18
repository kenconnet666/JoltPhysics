// tsc 不会把手写的声明文件带进产出：把 Emscripten 工厂的类型（src/*-core.d.mts）拷进 dist/，
// 和 CMake 放进去的 *-core.mjs 放在一起，使用方的 TypeScript 才能解析 dist/stride-native.d.ts 里的导入。
import { copyFileSync, readdirSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
for (const name of readdirSync(path.join(root, 'src'))) {
  if (name.endsWith('.d.mts')) copyFileSync(path.join(root, 'src', name), path.join(root, 'dist', name));
}
