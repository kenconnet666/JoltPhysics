// KTX2 (Basis Universal) transcoding to the GPU formats WebGPU can sample; mip levels in parallel.
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Texture.h>
#include <StrideNative/Plugin.h>

#include <basisu_transcoder.h>

#include <memory>
#include <mutex>

SN_PLUGIN(texture)

namespace
{
	void EnsureTranscoderInitialized()
	{
		static std::once_flag sOnce;
		std::call_once(sOnce, [] { basist::basisu_transcoder_init(); });
	}

	bool ToBasisFormat(int32_t inTarget, basist::transcoder_texture_format &outFormat)
	{
		switch (inTarget)
		{
		case SN_TEXTURE_RGBA8:			outFormat = basist::transcoder_texture_format::cTFRGBA32; return true;
		case SN_TEXTURE_BC7:			outFormat = basist::transcoder_texture_format::cTFBC7_RGBA; return true;
		case SN_TEXTURE_ASTC_4X4:		outFormat = basist::transcoder_texture_format::cTFASTC_LDR_4x4_RGBA; return true;
		case SN_TEXTURE_ETC2_RGBA8:		outFormat = basist::transcoder_texture_format::cTFETC2_RGBA; return true;
		default:						return false;
		}
	}

	// Shared by the per-level jobs: init and start_transcoding run once, levels in parallel, each
	// job with its own transcoder state (the transcoder is thread-safe only that way).
	struct Transcode
	{
		basist::ktx2_transcoder				mTranscoder;
		basist::transcoder_texture_format	mFormat;
	};
}

using namespace StrideNative;

SN_API int32_t sn_ktx2_get_info(const void *data, uint32_t size, sn_ktx2_info *out_info)
{
	if (data == nullptr || out_info == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "data and out_info are required");
	EnsureTranscoderInitialized();

	basist::ktx2_transcoder transcoder;
	if (!transcoder.init(data, size))
		return SetLastError(SN_ERROR_FAILED, "not a valid KTX2 file");

	out_info->width = transcoder.get_width();
	out_info->height = transcoder.get_height();
	out_info->levels = transcoder.get_levels();
	out_info->layers = transcoder.get_layers();
	out_info->faces = transcoder.get_faces();
	out_info->has_alpha = transcoder.get_has_alpha() != 0;
	out_info->is_srgb = transcoder.is_srgb();
	out_info->is_hdr = transcoder.is_hdr();
	return SN_OK;
}

SN_API int32_t sn_ktx2_transcode(const void *data, uint32_t size, int32_t target)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (data == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "data is null");
	EnsureTranscoderInitialized();

	auto transcode = std::make_shared<Transcode>();
	if (!ToBasisFormat(target, transcode->mFormat))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "unknown transcode target " + std::to_string(target));
	if (!transcode->mTranscoder.init(data, size))
		return SetLastError(SN_ERROR_FAILED, "not a valid KTX2 file");
	if (transcode->mTranscoder.is_hdr())
		return SetLastError(SN_ERROR_FAILED, "HDR KTX2 files are not supported by the LDR targets");
	if (!transcode->mTranscoder.start_transcoding())
		return SetLastError(SN_ERROR_FAILED, "KTX2 start_transcoding failed");

	uint32_t levels = transcode->mTranscoder.get_levels();
	return RunParallelTask("Ktx2Transcode", levels, [transcode](Task &ioTask, uint32_t inLevel)
	{
		basist::ktx2_image_level_info info;
		if (!transcode->mTranscoder.get_image_level_info(info, inLevel, 0, 0))
		{
			ioTask.Fail("cannot read KTX2 level " + std::to_string(inLevel));
			return;
		}

		uint32_t unit_size = basist::basis_get_bytes_per_block_or_pixel(transcode->mFormat);
		uint32_t units = basist::basis_transcoder_format_is_uncompressed(transcode->mFormat) ? info.m_orig_width * info.m_orig_height : info.m_total_blocks;
		void *output = ioTask.SetOutput(inLevel, units * unit_size);

		// The state has no constructor: clear() initializes the cached level index, without it a
		// garbage index can skip the Zstd decompression of the level.
		basist::ktx2_transcoder_state state;
		state.clear();
		if (!transcode->mTranscoder.transcode_image_level(inLevel, 0, 0, output, units, transcode->mFormat, 0, 0, 0, -1, -1, &state))
			ioTask.Fail("transcoding KTX2 level " + std::to_string(inLevel) + " failed");
	},
	[transcode](Task &ioTask)
	{
		ioTask.SetResult(0, int32_t(transcode->mTranscoder.get_width()));
		ioTask.SetResult(1, int32_t(transcode->mTranscoder.get_height()));
		ioTask.SetResult(2, int32_t(transcode->mTranscoder.get_levels()));
		ioTask.SetResult(3, int32_t(basist::basis_get_bytes_per_block_or_pixel(transcode->mFormat)));
		return true;
	}, levels);
}
