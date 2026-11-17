// Texture encoding (Basis Universal to KTX2) and MSDF font glyphs (stb_truetype + msdfgen).
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <basisu_comp.h>
#include <msdfgen.h>
#include <stb_truetype.h>

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

	// stb_truetype outlines in font units become a msdfgen shape. Quadratic and cubic segments map
	// one to one, so the distance field matches the glyph exactly.
	msdfgen::Shape GlyphShape(const stbtt_fontinfo &inFont, int inGlyph)
	{
		msdfgen::Shape shape;
		stbtt_vertex *vertices = nullptr;
		int count = stbtt_GetGlyphShape(&inFont, inGlyph, &vertices);
		msdfgen::Contour *contour = nullptr;
		msdfgen::Point2 pen;
		for (int i = 0; i < count; ++i)
		{
			const stbtt_vertex &v = vertices[i];
			msdfgen::Point2 to(v.x, v.y);
			switch (v.type)
			{
			case STBTT_vmove:
				contour = &shape.addContour();
				break;
			case STBTT_vline:
				contour->addEdge(msdfgen::EdgeHolder(pen, to));
				break;
			case STBTT_vcurve:
				contour->addEdge(msdfgen::EdgeHolder(pen, msdfgen::Point2(v.cx, v.cy), to));
				break;
			case STBTT_vcubic:
				contour->addEdge(msdfgen::EdgeHolder(pen, msdfgen::Point2(v.cx, v.cy), msdfgen::Point2(v.cx1, v.cy1), to));
				break;
			}
			pen = to;
		}
		stbtt_FreeShape(&inFont, vertices);
		return shape;
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
		// No cFlagThreaded: the encoder's own pool would oversubscribe the cores the job system owns;
		// textures are encoded in parallel as separate tasks instead.
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

SN_API int32_t st_font_msdf(const void *font, uint32_t font_size, const uint32_t *codepoints, uint32_t codepoint_count, float pixel_size, float distance_range)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (font == nullptr || codepoints == nullptr || pixel_size <= 0.0f || distance_range <= 0.0f)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "font, codepoints, pixel_size and distance_range are required");

	auto font_info = std::make_shared<stbtt_fontinfo>();
	const unsigned char *bytes = static_cast<const unsigned char *>(font);
	if (stbtt_InitFont(font_info.get(), bytes, stbtt_GetFontOffsetForIndex(bytes, 0)) == 0)
		return SetLastError(SN_ERROR_FAILED, "not a TrueType/OpenType font");
	(void)font_size; // stb_truetype trusts the table offsets; the size only documents the buffer

	// Output 0 holds the metrics of every glyph; glyph i's bitmap is output i + 1.
	auto metrics = std::make_shared<std::vector<st_glyph_metrics>>(codepoint_count);
	return RunParallelTask("FontMsdf", codepoint_count, [=](Task &ioTask, uint32_t inIndex)
	{
		const stbtt_fontinfo &info = *font_info;
		const float scale = stbtt_ScaleForMappingEmToPixels(&info, pixel_size);
		int glyph = stbtt_FindGlyphIndex(&info, int(codepoints[inIndex]));
		int advance = 0, left_bearing = 0;
		stbtt_GetGlyphHMetrics(&info, glyph, &advance, &left_bearing);

		st_glyph_metrics &m = (*metrics)[inIndex];
		m.codepoint = codepoints[inIndex];
		m.advance = advance * scale;

		int x0, y0, x1, y1;
		msdfgen::Shape shape = GlyphShape(info, glyph);
		if (shape.contours.empty() || stbtt_GetGlyphBox(&info, glyph, &x0, &y0, &x1, &y1) == 0)
			return; // Space and other glyphs without outline: metrics only

		// Pixel bounds of the glyph plus the distance range on every side.
		const float border = distance_range;
		const float left = std::floor(x0 * scale - border), bottom = std::floor(y0 * scale - border);
		m.width = uint32_t(std::ceil(x1 * scale + border) - left);
		m.height = uint32_t(std::ceil(y1 * scale + border) - bottom);
		m.bearing_x = left;
		m.bearing_y = bottom + float(m.height);

		shape.normalize();
		msdfgen::edgeColoringSimple(shape, 3.0);
		msdfgen::Bitmap<float, 3> bitmap(int(m.width), int(m.height));
		msdfgen::Projection projection(msdfgen::Vector2(scale), msdfgen::Vector2(-left / scale, -bottom / scale));
		msdfgen::generateMSDF(bitmap, shape, projection, msdfgen::Range(distance_range / scale));

		// msdfgen's rows go bottom-up; the bitmap we hand out goes top-down like an image.
		uint8_t *pixels = static_cast<uint8_t *>(ioTask.SetOutput(inIndex + 1, m.width * m.height * 3));
		for (uint32_t y = 0; y < m.height; ++y)
			for (uint32_t x = 0; x < m.width; ++x)
			{
				const float *source = bitmap(int(x), int(m.height - 1 - y));
				for (int channel = 0; channel < 3; ++channel)
					pixels[(y * m.width + x) * 3 + channel] = uint8_t(std::clamp(source[channel] * 255.0f + 0.5f, 0.0f, 255.0f));
			}
	},
	[metrics](Task &ioTask)
	{
		uint32_t size = uint32_t(metrics->size() * sizeof(st_glyph_metrics));
		if (size > 0)
			std::memcpy(ioTask.SetOutput(0, size), metrics->data(), size);
		return true;
	}, codepoint_count + 1);
}
