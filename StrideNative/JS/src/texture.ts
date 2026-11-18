// 贴图侧模块（KTX2 / Basis Universal 转码），StrideNative.ktx2Info、transcodeKtx2 要用：createStrideNative({ features: [texture] }) 时加载。
// 单独的入口，没导入它的应用，打包器不会输出它的 wasm。
import type { NativeFeature } from './native-module.ts';

export const texture: NativeFeature = {
	name: 'texture',
	url: new URL('./stride-native-texture.wasm', import.meta.url),
	debugUrl: new URL('./stride-native-texture-debug.wasm', import.meta.url),
};
