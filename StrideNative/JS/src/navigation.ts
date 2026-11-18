// 导航侧模块（Recast/Detour），StrideNative.buildNavMesh、loadNavMesh 要用：createStrideNative({ features: [navigation] }) 时加载。
// 单独的入口，没导入它的应用，打包器不会输出它的 wasm。
import type { NativeFeature } from './native-module.ts';

export const navigation: NativeFeature = {
	name: 'navigation',
	url: new URL('./stride-native-navigation.wasm', import.meta.url),
	debugUrl: new URL('./stride-native-navigation-debug.wasm', import.meta.url),
};
