// Fonts (stb_truetype): descriptions and name lookup, codepoint and kerning tables, glyph outlines as quadratic
// Bezier curves (the vector text renderer's input, cubic CFF segments split into quadratics), and MSDF glyphs (msdfgen).
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <msdfgen.h>
#include <stb_truetype.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace StrideNative;

namespace
{
	uint16_t ReadU16(const uint8_t *inData) { return uint16_t(inData[0] << 8 | inData[1]); }
	int16_t ReadS16(const uint8_t *inData) { return int16_t(ReadU16(inData)); }
	uint32_t ReadU32(const uint8_t *inData) { return uint32_t(inData[0]) << 24 | uint32_t(inData[1]) << 16 | uint32_t(inData[2]) << 8 | inData[3]; }

	/// A table stb_truetype does not expose, from the font's table directory (nullptr when missing).
	const uint8_t *FindTable(const stbtt_fontinfo &inFont, const char *inTag, uint32_t &outLength)
	{
		const uint8_t *start = inFont.data + inFont.fontstart;
		const uint16_t count = ReadU16(start + 4);
		for (uint16_t i = 0; i < count; ++i)
		{
			const uint8_t *record = start + 12 + 16 * i;
			if (std::memcmp(record, inTag, 4) == 0)
			{
				outLength = ReadU32(record + 12);
				return inFont.data + ReadU32(record + 8);
			}
		}
		outLength = 0;
		return nullptr;
	}

	bool InitFont(stbtt_fontinfo &outFont, const void *inData, uint32_t inSize, int32_t inIndex)
	{
		const unsigned char *bytes = static_cast<const unsigned char *>(inData);
		if (inData == nullptr || inSize < 12)
			return false;
		int offset = stbtt_GetFontOffsetForIndex(bytes, inIndex);
		return offset >= 0 && stbtt_InitFont(&outFont, bytes, offset) != 0;
	}

	/// A name table string (Windows Unicode, English US, then Macintosh Roman) as UTF-8 in a fixed buffer.
	void CopyName(const stbtt_fontinfo &inFont, int inNameId, char *outName, size_t inCapacity)
	{
		std::string utf8;
		int length = 0;
		if (const char *name = stbtt_GetFontNameString(&inFont, &length, STBTT_PLATFORM_ID_MICROSOFT, STBTT_MS_EID_UNICODE_BMP, STBTT_MS_LANG_ENGLISH, inNameId))
		{
			for (int i = 0; i + 1 < length; i += 2)
			{
				uint32_t c = uint32_t(uint8_t(name[i])) << 8 | uint8_t(name[i + 1]);
				if (c < 0x80)
					utf8 += char(c);
				else if (c < 0x800)
					utf8 += { char(0xC0 | c >> 6), char(0x80 | (c & 0x3F)) };
				else
					utf8 += { char(0xE0 | c >> 12), char(0x80 | (c >> 6 & 0x3F)), char(0x80 | (c & 0x3F)) };
			}
		}
		else if (const char *name = stbtt_GetFontNameString(&inFont, &length, STBTT_PLATFORM_ID_MAC, STBTT_MAC_EID_ROMAN, STBTT_MAC_LANG_ENGLISH, inNameId))
			utf8.assign(name, size_t(length));
		size_t count = std::min(utf8.size(), inCapacity - 1);
		std::memcpy(outName, utf8.data(), count);
		outName[count] = 0;
	}

	struct Point
	{
		float x, y;
	};

	/// Appends quadratic curves approximating a cubic within inTolerance (font units): the single quadratic through the
	/// endpoints whose control point is (3 (c1 + c2) - (p0 + p3)) / 4 is off by at most sqrt(3) / 36 |p3 - 3 c2 + 3 c1 - p0|;
	/// halves are split at t = 1/2 until that bound holds.
	void CubicToQuadratics(Point inP0, Point inC1, Point inC2, Point inP3, float inTolerance, std::vector<float> &ioCurves, int inDepth = 0)
	{
		const float dx = inP3.x - 3.0f * inC2.x + 3.0f * inC1.x - inP0.x, dy = inP3.y - 3.0f * inC2.y + 3.0f * inC1.y - inP0.y;
		if (inDepth >= 8 || std::sqrt(3.0f) / 36.0f * std::sqrt(dx * dx + dy * dy) <= inTolerance)
		{
			const Point c { (3.0f * (inC1.x + inC2.x) - inP0.x - inP3.x) / 4.0f, (3.0f * (inC1.y + inC2.y) - inP0.y - inP3.y) / 4.0f };
			ioCurves.insert(ioCurves.end(), { inP0.x, inP0.y, c.x, c.y, inP3.x, inP3.y });
			return;
		}
		auto mid = [](Point inA, Point inB) { return Point { (inA.x + inB.x) / 2.0f, (inA.y + inB.y) / 2.0f }; };
		const Point ab = mid(inP0, inC1), bc = mid(inC1, inC2), cd = mid(inC2, inP3), abc = mid(ab, bc), bcd = mid(bc, cd), center = mid(abc, bcd);
		CubicToQuadratics(inP0, ab, abc, center, inTolerance, ioCurves, inDepth + 1);
		CubicToQuadratics(center, bcd, cd, inP3, inTolerance, ioCurves, inDepth + 1);
	}

	/// The glyph's contours as quadratic curves (x1 y1 x2 y2 x3 y3, font units, y up). A line from p1 to p2 is {p1, p2, p2}
	/// (the encoding Slug recommends); zero-length segments are dropped.
	std::vector<float> GlyphCurves(const stbtt_fontinfo &inFont, int inGlyph, float inTolerance)
	{
		std::vector<float> curves;
		stbtt_vertex *vertices = nullptr;
		const int count = stbtt_GetGlyphShape(&inFont, inGlyph, &vertices);
		Point pen { 0.0f, 0.0f };
		for (int i = 0; i < count; ++i)
		{
			const stbtt_vertex &v = vertices[i];
			const Point to { float(v.x), float(v.y) };
			switch (v.type)
			{
			case STBTT_vline:
				if (to.x != pen.x || to.y != pen.y)
					curves.insert(curves.end(), { pen.x, pen.y, to.x, to.y, to.x, to.y });
				break;
			case STBTT_vcurve:
				curves.insert(curves.end(), { pen.x, pen.y, float(v.cx), float(v.cy), to.x, to.y });
				break;
			case STBTT_vcubic:
				CubicToQuadratics(pen, { float(v.cx), float(v.cy) }, { float(v.cx1), float(v.cy1) }, to, inTolerance, curves);
				break;
			default:
				break;
			}
			pen = to;
		}
		stbtt_FreeShape(&inFont, vertices);
		return curves;
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

SN_API int32_t st_font_describe(const void *font, uint32_t font_size, int32_t index, st_font_description *out_description)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (out_description == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "out_description is required");
	stbtt_fontinfo info;
	if (!InitFont(info, font, font_size, index))
		return SetLastError(SN_ERROR_FAILED, "not a TrueType/OpenType font (or no font at index " + std::to_string(index) + ")");

	st_font_description &d = *out_description;
	d = {};
	d.font_count = stbtt_GetNumberOfFonts(static_cast<const unsigned char *>(font));
	d.glyph_count = info.numGlyphs;
	uint32_t length = 0;
	if (const uint8_t *head = FindTable(info, "head", length); head != nullptr && length >= 54)
	{
		d.head_flags = ReadU16(head + 16);
		d.units_per_em = ReadU16(head + 18);
		for (int i = 0; i < 4; ++i)
			d.bounds[i] = ReadS16(head + 36 + 2 * i);
	}
	stbtt_GetFontVMetrics(&info, &d.ascent, &d.descent, &d.line_gap);
	if (const uint8_t *os2 = FindTable(info, "OS/2", length); os2 != nullptr && length >= 78)
	{
		d.use_typo_metrics = (ReadU16(os2 + 62) & 0x80) != 0 ? 1 : 0;
		d.typo_ascent = ReadS16(os2 + 68);
		d.typo_descent = ReadS16(os2 + 70);
		d.typo_line_gap = ReadS16(os2 + 72);
		d.win_ascent = ReadU16(os2 + 74);
		d.win_descent = ReadU16(os2 + 76);
		if (ReadU16(os2) >= 2 && length >= 90)
		{
			d.x_height = ReadS16(os2 + 86);
			d.cap_height = ReadS16(os2 + 88);
		}
	}
	CopyName(info, 1, d.family, sizeof(d.family));
	CopyName(info, 2, d.subfamily, sizeof(d.subfamily));
	CopyName(info, 16, d.typographic_family, sizeof(d.typographic_family));
	CopyName(info, 17, d.typographic_subfamily, sizeof(d.typographic_subfamily));
	return SN_OK;
}

SN_API int32_t st_font_codepoints(const void *font, uint32_t font_size, int32_t index)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	auto info = std::make_shared<stbtt_fontinfo>();
	if (!InitFont(*info, font, font_size, index))
		return SetLastError(SN_ERROR_FAILED, "not a TrueType/OpenType font (or no font at index " + std::to_string(index) + ")");

	return RunTask("FontCodepoints", [info](Task &ioTask)
	{
		std::vector<uint32_t> codepoints, glyphs;
		for (uint32_t codepoint = 0; codepoint <= 0x10FFFF; ++codepoint)
			if (int glyph = stbtt_FindGlyphIndex(info.get(), int(codepoint)); glyph != 0)
			{
				codepoints.push_back(codepoint);
				glyphs.push_back(uint32_t(glyph));
			}
		std::vector<stbtt_kerningentry> kerning(size_t(std::max(stbtt_GetKerningTableLength(info.get()), 0)));
		kerning.resize(size_t(stbtt_GetKerningTable(info.get(), kerning.data(), int(kerning.size()))));
		auto output = [&ioTask](const void *inData, size_t inBytes) { std::memcpy(ioTask.AddOutput(uint32_t(inBytes)), inData, inBytes); };
		output(codepoints.data(), codepoints.size() * 4);
		output(glyphs.data(), glyphs.size() * 4);
		output(kerning.data(), kerning.size() * sizeof(stbtt_kerningentry));
		return true;
	});
}

SN_API int32_t st_font_outlines(const void *font, uint32_t font_size, int32_t index, const uint32_t *glyphs, uint32_t glyph_count, float tolerance)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (glyphs == nullptr && glyph_count > 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "glyphs is required");
	if (!(tolerance > 0.0f))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "tolerance must be positive");
	auto info = std::make_shared<stbtt_fontinfo>();
	if (!InitFont(*info, font, font_size, index))
		return SetLastError(SN_ERROR_FAILED, "not a TrueType/OpenType font (or no font at index " + std::to_string(index) + ")");

	auto outlines = std::make_shared<std::vector<st_glyph_outline>>(glyph_count);
	auto curves = std::make_shared<std::vector<std::vector<float>>>(glyph_count);
	return RunParallelTask("FontOutlines", glyph_count, [=](Task &, uint32_t inIndex)
	{
		const int glyph = int(glyphs[inIndex]);
		st_glyph_outline &o = (*outlines)[inIndex];
		o.glyph = uint32_t(glyph);
		stbtt_GetGlyphHMetrics(info.get(), glyph, &o.advance, &o.left_bearing);
		(*curves)[inIndex] = GlyphCurves(*info, glyph, tolerance);
		if (!(*curves)[inIndex].empty())
			stbtt_GetGlyphBox(info.get(), glyph, &o.bounds[0], &o.bounds[1], &o.bounds[2], &o.bounds[3]);
	},
	[outlines, curves](Task &ioTask)
	{
		uint32_t total = 0;
		for (size_t i = 0; i < outlines->size(); ++i)
		{
			(*outlines)[i].curve_offset = total;
			(*outlines)[i].curve_count = uint32_t((*curves)[i].size() / 6);
			total += (*outlines)[i].curve_count;
		}
		std::memcpy(ioTask.AddOutput(uint32_t(outlines->size() * sizeof(st_glyph_outline))), outlines->data(), outlines->size() * sizeof(st_glyph_outline));
		float *out = static_cast<float *>(ioTask.AddOutput(total * 6 * sizeof(float)));
		for (const std::vector<float> &glyph_curves : *curves)
			out = std::copy(glyph_curves.begin(), glyph_curves.end(), out);
		return true;
	});
}

SN_API int32_t st_font_msdf(const void *font, uint32_t font_size, const uint32_t *codepoints, uint32_t codepoint_count, float pixel_size, float distance_range)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (font == nullptr || codepoints == nullptr || pixel_size <= 0.0f || distance_range <= 0.0f)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "font, codepoints, pixel_size and distance_range are required");

	auto font_info = std::make_shared<stbtt_fontinfo>();
	if (!InitFont(*font_info, font, font_size, 0))
		return SetLastError(SN_ERROR_FAILED, "not a TrueType/OpenType font");

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
