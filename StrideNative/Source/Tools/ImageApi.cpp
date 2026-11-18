// 图片解码：PNG、JPEG、TGA（stb_image），统一变成 RGBA8，给 Basis 编码和管线里的其他处理用。
// DDS 这类已经是 GPU 格式的文件以后另做。
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <stb_image.h>

#include <cstring>

using namespace StrideNative;

SN_API int32_t st_image_decode(const void *data, uint32_t size)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (data == nullptr || size == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "image data is empty");

	return RunTask("ImageDecode", [data, size](Task &ioTask)
	{
		int width = 0, height = 0, channels = 0;
		stbi_uc *pixels = stbi_load_from_memory(static_cast<const stbi_uc *>(data), int(size), &width, &height, &channels, 4);
		if (pixels == nullptr)
		{
			ioTask.Fail(std::string("image decode failed: ") + stbi_failure_reason());
			return false;
		}
		uint32_t bytes = uint32_t(width) * uint32_t(height) * 4;
		std::memcpy(ioTask.AddOutput(bytes), pixels, bytes);
		stbi_image_free(pixels);
		ioTask.SetResult(0, width);
		ioTask.SetResult(1, height);
		ioTask.SetResult(2, channels);
		return true;
	});
}
