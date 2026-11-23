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
		std::call_once(sOnce, [] { basisu::basisu_encoder_init(); });
	}

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
		// By default no cFlagThreaded: the encoder's own pool would oversubscribe the cores the job system owns;
		// textures are encoded in parallel as separate tasks instead. A lone big texture opts in with ST_BASIS_FLAG_THREADED.
		if ((flags & ST_BASIS_FLAG_THREADED) != 0)
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
