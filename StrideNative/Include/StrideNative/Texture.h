// 贴图侧模块（stride-native-texture）的 C API：KTX2 / Basis Universal 转码。
#pragma once

#include <StrideNative/Core.h>

// ---------------------------------------------------------------------------------------------
// KTX2 / Basis Universal transcoding
// ---------------------------------------------------------------------------------------------

/// Transcode targets, named after the WebGPU format they upload to.
enum
{
	SN_TEXTURE_RGBA8 = 0,			///< rgba8unorm(-srgb)
	SN_TEXTURE_BC7 = 1,				///< bc7-rgba-unorm(-srgb), texture-compression-bc
	SN_TEXTURE_ASTC_4X4 = 2,		///< astc-4x4-unorm(-srgb), texture-compression-astc
	SN_TEXTURE_ETC2_RGBA8 = 3,		///< etc2-rgba8unorm(-srgb), texture-compression-etc2
};

typedef struct sn_ktx2_info
{
	uint32_t width;
	uint32_t height;
	uint32_t levels;
	uint32_t layers;				///< 0 for a plain 2D texture
	uint32_t faces;					///< 6 for cubemaps
	int32_t has_alpha;
	int32_t is_srgb;
	int32_t is_hdr;
} sn_ktx2_info;

SN_API int32_t sn_ktx2_get_info(const void *data, uint32_t size, sn_ktx2_info *out_info);

/// Transcodes every mip level in parallel. data must stay alive until the task finishes.
/// Outputs: one buffer per level (level 0 first) for layer 0 / face 0.
/// Results: [0] width, [1] height, [2] level count, [3] bytes per block (or per pixel for RGBA8).
SN_API int32_t sn_ktx2_transcode(const void *data, uint32_t size, int32_t target);
