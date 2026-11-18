// Emscripten 生成的 stride-native-core.mjs 的类型：模块工厂和运行时导出的 C 函数（Runtime.h、Plugin.h），加上内置
// 侧模块的（Physics.h、Texture.h、Navigation.h）：侧模块加载后它们的导出挂在同一个模块对象上，没加载时不存在。
// 函数的列表和签名从头文件生成，见 native-exports.ts。
import type { CoreFactory } from './native-module.js';
import type { RuntimeExport } from './native-exports.js';

export type { RuntimeExport };

declare const factory: CoreFactory<RuntimeExport>;
export default factory;
