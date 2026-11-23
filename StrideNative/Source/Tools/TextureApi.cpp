// Texture encoding (Basis Universal to KTX2).
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <basisu_comp.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>

namespace
{
	using namespace StrideNative;

	void EnsureEncoderInitialized()
	{
		static std::once_flag sOnce;
		std::call_once(sOnce, []
		{
			basisu::basisu_encoder_init();
			// A threaded encode builds its own pool of this many threads. Uncapped it is hardware_concurrency (36 on an
			// 18-core Xeon) per encode, all beyond the Emscripten pool the job system already fills, so every encode
			// spawned dozens of new Workers and concurrent encodes under load hung (a worker died with "NaN cannot be
			// converted to a BigInt"). As many as the job system has: the Workers spawn once and are reused.
			basisu::set_num_wasi_threads(uint32_t(std::max(1, GetJobSystem().GetMaxConcurrency() - 1)));
		});
	}

	/**
	 * Held by the one encode that runs on Basis's own pool. Each threaded encode brings a pool of its own, so concurrent
	 * ones would multiply the threads; the others encode single-threaded in their job-system task (a batch is
	 * parallel across textures anyway). Waiting for the pool instead made 24 concurrent 512² encodes 3x slower.
	 */
	std::mutex sThreadedEncode;

}

SN_API int32_t st_basis_encode(const uint8_t *rgba, uint32_t width, uint32_t height, int32_t mode, int32_t flags, int32_t quality, int32_t effort)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (rgba == nullptr || width == 0 || height == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "image data and size are required");
	if (mode != ST_BASIS_UASTC && mode != ST_BASIS_ETC1S)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "unknown basis mode " + std::to_string(mode));
	EnsureEncoderInitialized();

	return RunTask("BasisEncode", [=](Task &ioTask)
	{
		uint32_t encoder_flags = basisu::cFlagKTX2 | basisu::cFlagKTX2UASTCSuperCompression;
		if ((flags & ST_BASIS_FLAG_SRGB) != 0)
			encoder_flags |= basisu::cFlagSRGB;
		if ((flags & ST_BASIS_FLAG_MIPMAPS) != 0)
			encoder_flags |= basisu::cFlagGenMipsClamp;
		// ST_BASIS_FLAG_THREADED: use Basis's own pool if no other encode has it (see sThreadedEncode)
		std::unique_lock<std::mutex> threaded(sThreadedEncode, std::defer_lock);
		if ((flags & ST_BASIS_FLAG_THREADED) != 0 && threaded.try_lock())
			encoder_flags |= basisu::cFlagThreaded;
		basist::basis_tex_format format = mode == ST_BASIS_UASTC ? basist::basis_tex_format::cUASTC_LDR_4x4 : basist::basis_tex_format::cETC1S;

		size_t size = 0;
		void *ktx2 = basisu::basis_compress2(format, rgba, width, height, width, encoder_flags, std::clamp(quality, 0, 100), std::clamp(effort, 0, 10), &size);
		if (ktx2 == nullptr)
		{
			ioTask.Fail("Basis encoding failed");
			return false;
		}
		std::memcpy(ioTask.AddOutput(uint32_t(size)), ktx2, size);
		basisu::basis_free_data(ktx2);
		return true;
	});
}
