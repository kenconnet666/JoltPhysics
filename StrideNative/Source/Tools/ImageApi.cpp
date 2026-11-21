// 图片解码：PNG、JPEG、TGA（stb_image）和 DDS（只取最大的一级，块用 basisu 的解码器），统一变成 RGBA8，给 Basis 编码和
// 管线里的其他处理用；缩放用 basisu 的重采样器。
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <basisu_enc.h>
#include <basisu_gpu_texture.h>
#include <stb_image.h>
#include <webp/decode.h>

#include <cstring>
#include <string>
#include <vector>

using namespace StrideNative;

namespace
{
	uint32_t ReadU32(const uint8_t *p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }

	constexpr uint32_t FourCC(char a, char b, char c, char d)
	{
		return uint32_t(uint8_t(a)) | uint32_t(uint8_t(b)) << 8 | uint32_t(uint8_t(c)) << 16 | uint32_t(uint8_t(d)) << 24;
	}

	/// 掩码对应的通道值，按掩码的位数换到 0..255；掩码是 0 时返回 fallback。
	uint8_t Channel(uint32_t pixel, uint32_t mask, uint8_t fallback)
	{
		if (mask == 0)
			return fallback;
		uint32_t shift = 0;
		while (((mask >> shift) & 1) == 0)
			shift++;
		uint32_t max = mask >> shift;
		return uint8_t((((pixel & mask) >> shift) * 255 + max / 2) / max);
	}

	/// BC2（DXT3）：每像素 4 位的透明度，后面跟着 BC1 的颜色块（basisu 没有 BC2）。
	void UnpackBC2(const uint8_t *block, basisu::color_rgba *pixels)
	{
		basisu::unpack_block(basisu::texture_format::cBC1, block + 8, pixels, false);
		for (int i = 0; i < 16; i++)
			pixels[i].a = uint8_t(((block[i / 2] >> ((i & 1) * 4)) & 15) * 17);
	}

	/// DDS 文件的第一层（最大的那一级）解成 RGBA8：DXT1/3/5、BC4、BC5、BC7（DX10 头）和不压缩的 24、32 位格式。
	bool DecodeDds(const uint8_t *data, uint32_t size, std::vector<uint8_t> &outPixels, uint32_t &outWidth, uint32_t &outHeight, std::string &outError)
	{
		if (size < 128)
		{
			outError = "DDS header is truncated";
			return false;
		}
		const uint32_t height = ReadU32(data + 12), width = ReadU32(data + 16);
		const uint32_t pixelFlags = ReadU32(data + 80), fourCC = ReadU32(data + 84), bitCount = ReadU32(data + 88);
		uint32_t offset = 128;
		basisu::texture_format format = basisu::texture_format::cInvalidTextureFormat;
		bool bc2 = false;
		if (pixelFlags & 4) // DDPF_FOURCC
		{
			if (fourCC == FourCC('D', 'X', 'T', '1')) format = basisu::texture_format::cBC1;
			else if (fourCC == FourCC('D', 'X', 'T', '3') || fourCC == FourCC('D', 'X', 'T', '2')) bc2 = true;
			else if (fourCC == FourCC('D', 'X', 'T', '5') || fourCC == FourCC('D', 'X', 'T', '4')) format = basisu::texture_format::cBC3;
			else if (fourCC == FourCC('A', 'T', 'I', '1') || fourCC == FourCC('B', 'C', '4', 'U')) format = basisu::texture_format::cBC4;
			else if (fourCC == FourCC('A', 'T', 'I', '2') || fourCC == FourCC('B', 'C', '5', 'U')) format = basisu::texture_format::cBC5;
			else if (fourCC == FourCC('D', 'X', '1', '0'))
			{
				if (size < 148)
				{
					outError = "DDS DX10 header is truncated";
					return false;
				}
				offset = 148;
				const uint32_t dxgi = ReadU32(data + 128);
				switch (dxgi)
				{
				case 71: case 72: format = basisu::texture_format::cBC1; break;
				case 74: case 75: bc2 = true; break;
				case 77: case 78: format = basisu::texture_format::cBC3; break;
				case 80: format = basisu::texture_format::cBC4; break;
				case 83: format = basisu::texture_format::cBC5; break;
				case 98: case 99: format = basisu::texture_format::cBC7; break;
				default:
					outError = "DDS DXGI format " + std::to_string(dxgi) + " is not supported";
					return false;
				}
			}
			else
			{
				outError = "DDS four-character code is not supported";
				return false;
			}
		}
		else if ((pixelFlags & 0x40) == 0 || (bitCount != 32 && bitCount != 24)) // DDPF_RGB
		{
			outError = "DDS pixel format is not supported (only BC1-BC5, BC7 and 24/32-bit RGB)";
			return false;
		}

		outWidth = width;
		outHeight = height;
		outPixels.assign(size_t(width) * height * 4, 0);
		if (format == basisu::texture_format::cInvalidTextureFormat && !bc2)
		{
			const uint32_t stride = bitCount / 8;
			if (uint64_t(offset) + uint64_t(width) * height * stride > size)
			{
				outError = "DDS pixel data is truncated";
				return false;
			}
			const uint32_t r = ReadU32(data + 92), g = ReadU32(data + 96), b = ReadU32(data + 100), a = (pixelFlags & 1) ? ReadU32(data + 104) : 0;
			for (uint32_t i = 0; i < width * height; i++)
			{
				const uint8_t *p = data + offset + i * stride;
				const uint32_t pixel = stride == 4 ? ReadU32(p) : uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16;
				uint8_t *out = &outPixels[size_t(i) * 4];
				out[0] = Channel(pixel, r, 0);
				out[1] = Channel(pixel, g, 0);
				out[2] = Channel(pixel, b, 0);
				out[3] = Channel(pixel, a, 255);
			}
			return true;
		}

		const uint32_t blockBytes = format == basisu::texture_format::cBC1 || format == basisu::texture_format::cBC4 ? 8 : 16;
		const uint32_t blocksX = (width + 3) / 4, blocksY = (height + 3) / 4;
		if (uint64_t(offset) + uint64_t(blocksX) * blocksY * blockBytes > size)
		{
			outError = "DDS block data is truncated";
			return false;
		}
		basisu::color_rgba block[16];
		for (uint32_t by = 0; by < blocksY; by++)
			for (uint32_t bx = 0; bx < blocksX; bx++)
			{
				const uint8_t *source = data + offset + (size_t(by) * blocksX + bx) * blockBytes;
				if (bc2)
					UnpackBC2(source, block);
				else if (!basisu::unpack_block(format, source, block, false))
				{
					outError = "DDS block decode failed";
					return false;
				}
				for (uint32_t y = 0; y < 4 && by * 4 + y < height; y++)
					for (uint32_t x = 0; x < 4 && bx * 4 + x < width; x++)
					{
						basisu::color_rgba c = block[y * 4 + x];
						// BC4、BC5 只有红（和绿）通道
						if (format == basisu::texture_format::cBC4)
							c = basisu::color_rgba(c.r, c.r, c.r, 255);
						else if (format == basisu::texture_format::cBC5)
							c = basisu::color_rgba(c.r, c.g, 0, 255);
						std::memcpy(&outPixels[((size_t(by) * 4 + y) * width + bx * 4 + x) * 4], &c, 4);
					}
			}
		return true;
	}
}

SN_API int32_t st_image_decode(const void *data, uint32_t size)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (data == nullptr || size == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "image data is empty");

	return RunTask("ImageDecode", [data, size](Task &ioTask)
	{
		const uint8_t *file = static_cast<const uint8_t *>(data);
		if (size >= 4 && ReadU32(file) == FourCC('D', 'D', 'S', ' '))
		{
			std::vector<uint8_t> rgba;
			uint32_t ddsWidth = 0, ddsHeight = 0;
			std::string error;
			if (!DecodeDds(file, size, rgba, ddsWidth, ddsHeight, error))
			{
				ioTask.Fail("image decode failed: " + error);
				return false;
			}
			std::memcpy(ioTask.AddOutput(uint32_t(rgba.size())), rgba.data(), rgba.size());
			ioTask.SetResult(0, int32_t(ddsWidth));
			ioTask.SetResult(1, int32_t(ddsHeight));
			ioTask.SetResult(2, 4);
			return true;
		}
		if (size >= 12 && ReadU32(file) == FourCC('R', 'I', 'F', 'F') && ReadU32(file + 8) == FourCC('W', 'E', 'B', 'P'))
		{
			// WebP（有损、无损、带透明）：stb_image 不认，用 libwebp 的解码器
			WebPBitstreamFeatures features;
			int webpWidth = 0, webpHeight = 0;
			uint8_t *rgba = WebPGetFeatures(file, size, &features) == VP8_STATUS_OK ? WebPDecodeRGBA(file, size, &webpWidth, &webpHeight) : nullptr;
			if (rgba == nullptr)
			{
				ioTask.Fail("image decode failed: not a valid WebP image");
				return false;
			}
			uint32_t bytes = uint32_t(webpWidth) * uint32_t(webpHeight) * 4;
			std::memcpy(ioTask.AddOutput(bytes), rgba, bytes);
			WebPFree(rgba);
			ioTask.SetResult(0, webpWidth);
			ioTask.SetResult(1, webpHeight);
			ioTask.SetResult(2, features.has_alpha ? 4 : 3);
			return true;
		}
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

SN_API int32_t st_image_resize(const uint8_t *rgba, uint32_t width, uint32_t height, uint32_t out_width, uint32_t out_height)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (rgba == nullptr || width == 0 || height == 0 || out_width == 0 || out_height == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "image data and both sizes are required");

	return RunTask("ImageResize", [=](Task &ioTask)
	{
		basisu::image source(rgba, width, height, 4);
		basisu::image resized(out_width, out_height);
		if (!basisu::image_resample(source, resized, false, "lanczos3"))
		{
			ioTask.Fail("image resize failed: " + std::to_string(width) + "x" + std::to_string(height) + " to " + std::to_string(out_width) + "x" + std::to_string(out_height));
			return false;
		}
		uint32_t bytes = out_width * out_height * 4;
		std::memcpy(ioTask.AddOutput(bytes), resized.get_ptr(), bytes);
		return true;
	});
}
