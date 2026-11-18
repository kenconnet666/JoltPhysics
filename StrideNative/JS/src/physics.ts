// 物理侧模块（Jolt），StrideNative.createPhysicsWorld 要用：createStrideNative({ features: [physics] }) 时加载。
// 单独的入口，没导入它的应用，打包器不会输出它的 wasm。
import type { NativeFeature } from './native-module.ts';

export const physics: NativeFeature = {
	name: 'physics',
	url: new URL('./stride-native-physics.wasm', import.meta.url),
	debugUrl: new URL('./stride-native-physics-debug.wasm', import.meta.url),
};
