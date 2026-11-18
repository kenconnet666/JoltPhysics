// 天空盒的光照预处理，和 Stride 的 SkyboxGenerator 一致：
// - 漫反射：辐射度投影到 3 阶球谐，乘余弦核（Lambert 预滤波）和基函数常数（LambertianPrefilteringSHNoCompute）；
// - 镜面：GGX 重要性采样预滤波出一张带 mip 的立方体贴图，第 i 级的粗糙度是 i / (级数 - 1)（RadiancePrefilteringGGXNoCompute）。
// 立方体贴图的布局同 D3D/WebGPU：面的顺序 +X -X +Y -Y +Z -Z，每面的行从上到下，RGBA32F。
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

using namespace StrideNative;

namespace
{
	constexpr float PI = 3.14159265358979f;

	struct Vec3
	{
		float x, y, z;
	};

	Vec3 Normalize(Vec3 v)
	{
		float length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
		return { v.x / length, v.y / length, v.z / length };
	}

	float Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	Vec3 Cross(Vec3 a, Vec3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }

	/// 面上的坐标（-1..1，v 向下）到方向，同 Stride 的 CubemapUtils.ConvertTexcoordsNoFlip。
	Vec3 FaceDirection(int face, float u, float v)
	{
		switch (face)
		{
		case 0: return { 1, -v, -u };
		case 1: return { -1, -v, u };
		case 2: return { u, 1, v };
		case 3: return { u, -1, -v };
		case 4: return { u, -v, 1 };
		default: return { -u, -v, -1 };
		}
	}

	/// 带完整 mip 链的立方体贴图，每级 6 个面连续存放。
	class Cubemap
	{
	public:
		Cubemap(const float *inFaces, uint32_t inSize) : mSize(inSize)
		{
			uint32_t levels = 1;
			while ((inSize >> (levels - 1)) > 1)
				++levels;
			mLevels.resize(levels);
			mLevels[0].assign(inFaces, inFaces + size_t(6) * inSize * inSize * 4);
			// 2x2 平均生成下面的 mip（同 Stride 的 GenerateMipmaps：精确的 2 倍缩小，一次双线性采样）
			for (uint32_t level = 1; level < levels; ++level)
			{
				uint32_t size = SizeOf(level), source = SizeOf(level - 1);
				mLevels[level].resize(size_t(6) * size * size * 4);
				for (int face = 0; face < 6; ++face)
					for (uint32_t y = 0; y < size; ++y)
						for (uint32_t x = 0; x < size; ++x)
							for (int c = 0; c < 4; ++c)
							{
								auto at = [&](uint32_t sx, uint32_t sy) { return mLevels[level - 1][((size_t(face) * source + sy) * source + sx) * 4 + c]; };
								mLevels[level][((size_t(face) * size + y) * size + x) * 4 + c] = 0.25f * (at(2 * x, 2 * y) + at(2 * x + 1, 2 * y) + at(2 * x, 2 * y + 1) + at(2 * x + 1, 2 * y + 1));
							}
			}
		}

		uint32_t Size() const { return mSize; }
		uint32_t LevelCount() const { return uint32_t(mLevels.size()); }
		uint32_t SizeOf(uint32_t inLevel) const { return std::max(1u, mSize >> inLevel); }

		/// 三线性采样（每个面内双线性、钳在边上；mip 之间线性）。
		void Sample(Vec3 inDirection, float inLevel, float outColor[4]) const
		{
			inLevel = std::clamp(inLevel, 0.0f, float(LevelCount() - 1));
			uint32_t low = uint32_t(inLevel);
			uint32_t high = std::min(low + 1, LevelCount() - 1);
			float t = inLevel - float(low);
			float a[4], b[4];
			SampleLevel(inDirection, low, a);
			SampleLevel(inDirection, high, b);
			for (int c = 0; c < 4; ++c)
				outColor[c] = a[c] + (b[c] - a[c]) * t;
		}

		/// 某一级 mip 的双线性采样。
		void SampleLevel(Vec3 d, uint32_t inLevel, float outColor[4]) const
		{
			float ax = std::abs(d.x), ay = std::abs(d.y), az = std::abs(d.z);
			int face;
			float u, v;
			if (ax >= ay && ax >= az)
			{
				face = d.x > 0 ? 0 : 1;
				u = (d.x > 0 ? -d.z : d.z) / ax;
				v = -d.y / ax;
			}
			else if (ay >= az)
			{
				face = d.y > 0 ? 2 : 3;
				u = d.x / ay;
				v = (d.y > 0 ? d.z : -d.z) / ay;
			}
			else
			{
				face = d.z > 0 ? 4 : 5;
				u = (d.z > 0 ? d.x : -d.x) / az;
				v = -d.y / az;
			}
			uint32_t size = SizeOf(inLevel);
			float fx = std::clamp((u + 1) * 0.5f * size - 0.5f, 0.0f, float(size - 1));
			float fy = std::clamp((v + 1) * 0.5f * size - 0.5f, 0.0f, float(size - 1));
			uint32_t x0 = uint32_t(fx), y0 = uint32_t(fy);
			uint32_t x1 = std::min(x0 + 1, size - 1), y1 = std::min(y0 + 1, size - 1);
			float tx = fx - float(x0), ty = fy - float(y0);
			const float *texels = mLevels[inLevel].data() + size_t(face) * size * size * 4;
			for (int c = 0; c < 4; ++c)
			{
				float top = texels[(y0 * size + x0) * 4 + c] * (1 - tx) + texels[(y0 * size + x1) * 4 + c] * tx;
				float bottom = texels[(y1 * size + x0) * 4 + c] * (1 - tx) + texels[(y1 * size + x1) * 4 + c] * tx;
				outColor[c] = top * (1 - ty) + bottom * ty;
			}
		}

	private:
		uint32_t mSize;
		std::vector<std::vector<float>> mLevels;
	};

	/// Hammersley 点集，同 Stride 的 Hammersley.GetSamplePlane。
	void Hammersley(uint32_t inIndex, uint32_t inCount, float &outU, float &outV)
	{
		float u = 0, p = 0.5f;
		for (uint32_t k = inIndex; k > 0; p *= 0.5f, k >>= 1)
			if (k & 1)
				u += p;
		outU = u;
		outV = (float(inIndex) + 0.5f) / float(inCount);
	}

	/// GGX 重要性采样的半角向量，同 Stride 的 ImportanceSamplingGGX.GetSample（a = 粗糙度²）。
	Vec3 SampleGGX(float u, float v, float inRoughness, Vec3 n)
	{
		float a = inRoughness * inRoughness;
		float phi = 2 * PI * u;
		float cosTheta = std::sqrt((1 - v) / (1 + (a * a - 1) * v));
		float sinTheta = std::sqrt(1 - cosTheta * cosTheta);
		Vec3 up = std::abs(n.z) < 0.999f ? Vec3 { 0, 0, 1 } : Vec3 { 1, 0, 0 };
		Vec3 tangentX = Normalize(Cross(up, n));
		Vec3 tangentY = Cross(n, tangentX);
		float hx = sinTheta * std::cos(phi), hy = sinTheta * std::sin(phi);
		return { tangentX.x * hx + tangentY.x * hy + n.x * cosTheta, tangentX.y * hx + tangentY.y * hy + n.y * cosTheta, tangentX.z * hx + tangentY.z * hy + n.z * cosTheta };
	}

	float DistributionGGX(float alpha, float nDotH)
	{
		float alpha2 = alpha * alpha;
		float d = std::max(nDotH * nDotH * (alpha2 - 1) + 1, 0.0001f);
		return alpha2 / (PI * d * d);
	}

	/// 预滤波一个输出纹素，同 RadiancePrefilteringGGXNoComputeShader（包括它按 NoL 算 pdf 的写法，保持结果一致）。
	void PrefilterTexel(const Cubemap &inSource, Vec3 r, float inRoughness, uint32_t inSamples, float outColor[4])
	{
		float sum[4] = {};
		const float omegaP = 4 * PI / (6.0f * float(inSource.Size()) * float(inSource.Size()));
		const float maxLevel = float(inSource.LevelCount() - 1);
		for (uint32_t i = 0; i < inSamples; ++i)
		{
			float u, v;
			Hammersley(i, inSamples, u, v);
			Vec3 h = SampleGGX(u, v, inRoughness, r);
			float rDotH = Dot(r, h);
			Vec3 l = { 2 * rDotH * h.x - r.x, 2 * rDotH * h.y - r.y, 2 * rDotH * h.z - r.z };
			float nDotL = std::clamp(Dot(r, l), 0.0f, 1.0f);
			if (nDotL <= 0)
				continue;
			float pdf = DistributionGGX(inRoughness * inRoughness, nDotL) / 4;
			float omegaS = 1.0f / (float(inSamples) * pdf);
			float level = std::clamp(0.5f * std::log2(omegaS / omegaP), 0.0f, maxLevel);
			float color[4];
			inSource.Sample(l, level, color);
			for (int c = 0; c < 3; ++c)
				sum[c] += color[c] * nDotL;
			sum[3] += nDotL;
		}
		for (int c = 0; c < 3; ++c)
			outColor[c] = sum[3] > 0 ? sum[c] / sum[3] : 0;
		outColor[3] = 1;
	}

	/// 3 阶球谐的基函数（不带常数和符号，同 Stride 的 EvaluateSphericalHarmonics）。
	void ShPolynomials(Vec3 d, float outValues[9])
	{
		outValues[0] = 1;
		outValues[1] = d.y;
		outValues[2] = d.z;
		outValues[3] = d.x;
		outValues[4] = d.y * d.x;
		outValues[5] = d.y * d.z;
		outValues[6] = 3 * d.z * d.z - 1;
		outValues[7] = d.x * d.z;
		outValues[8] = d.x * d.x - d.y * d.y;
	}

	/// 基函数常数（带符号），同 Stride 的 SphericalHarmonics.BaseCoefficients。
	const float SH_BASE[9] = {
		0.282094792f,
		-0.488602512f, 0.488602512f, -0.488602512f,
		1.092548431f, -1.092548431f, 0.315391565f, -1.092548431f, 0.546274215f,
	};

	/// Lambert 余弦核在各阶的系数，同 LambertianPrefilteringSHNoComputePass1 的 A0..A2。
	const float SH_COSINE[9] = { 1.0f, 2.0f / 3, 2.0f / 3, 2.0f / 3, 0.25f, 0.25f, 0.25f, 0.25f, 0.25f };

	bool IsPowerOfTwo(uint32_t x) { return x != 0 && (x & (x - 1)) == 0; }
}

SN_API int32_t st_cubemap_prefilter_ggx(const float *faces, uint32_t size, uint32_t out_size, uint32_t samples)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (faces == nullptr || !IsPowerOfTwo(size) || !IsPowerOfTwo(out_size) || samples == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "faces, power-of-two size and out_size, and samples are required");

	auto source = std::make_shared<const Cubemap>(faces, size);
	uint32_t levels = 1;
	while ((out_size >> (levels - 1)) > 1)
		++levels;
	// 每一级的起点（以 float 计），所有级连在一起放进输出 0
	auto offsets = std::make_shared<std::vector<size_t>>(levels + 1, 0);
	for (uint32_t level = 0; level < levels; ++level)
	{
		size_t edge = std::max(1u, out_size >> level);
		(*offsets)[level + 1] = (*offsets)[level] + 6 * edge * edge * 4;
	}
	auto output = std::make_shared<std::vector<float>>(offsets->back());

	// 并行项是（级别，面，行）
	struct Row
	{
		uint32_t level, face, y;
	};
	auto rows = std::make_shared<std::vector<Row>>();
	for (uint32_t level = 0; level < levels; ++level)
		for (uint32_t face = 0; face < 6; ++face)
			for (uint32_t y = 0; y < std::max(1u, out_size >> level); ++y)
				rows->push_back({ level, face, y });

	return RunParallelTask("CubemapPrefilterGGX", uint32_t(rows->size()), [=](Task &, uint32_t inIndex)
	{
		const Row row = (*rows)[inIndex];
		const uint32_t edge = std::max(1u, out_size >> row.level);
		const float roughness = levels > 1 ? float(row.level) / float(levels - 1) : 0.0f;
		float *texels = output->data() + (*offsets)[row.level] + (size_t(row.face) * edge + row.y) * edge * 4;
		for (uint32_t x = 0; x < edge; ++x)
		{
			Vec3 r = Normalize(FaceDirection(int(row.face), (float(x) + 0.5f) / float(edge) * 2 - 1, (float(row.y) + 0.5f) / float(edge) * 2 - 1));
			if (row.level == 0)
			{
				// 最高一级不滤波，按方向重采样源贴图最接近的一级（Stride 的 DoNotFilterHighestLevel）
				float sourceLevel = std::max(0.0f, std::log2(float(size) / float(edge)));
				source->SampleLevel(r, uint32_t(std::min(sourceLevel, float(source->LevelCount() - 1))), texels + x * 4);
				texels[x * 4 + 3] = 1;
			}
			else
				PrefilterTexel(*source, r, roughness, samples, texels + x * 4);
		}
	},
	[output](Task &ioTask)
	{
		uint32_t bytes = uint32_t(output->size() * sizeof(float));
		std::memcpy(ioTask.AddOutput(bytes), output->data(), bytes);
		return true;
	});
}

SN_API int32_t st_cubemap_sh_lambert(const float *faces, uint32_t size)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (faces == nullptr || size == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "faces and size are required");

	return RunTask("CubemapShLambert", [faces, size](Task &ioTask)
	{
		// 每个纹素按它占的立体角加权：4 / (sqrt(d) * d)，d = 1 + u² + v²
		double sums[9][3] = {}, weightSum = 0;
		for (uint32_t y = 0; y < size; ++y)
			for (uint32_t x = 0; x < size; ++x)
			{
				float u = (float(x) + 0.5f) / float(size) * 2 - 1, v = (float(y) + 0.5f) / float(size) * 2 - 1;
				float dist = 1 + u * u + v * v;
				float weight = 4 / (std::sqrt(dist) * dist);
				weightSum += 6.0 * weight;
				for (int face = 0; face < 6; ++face)
				{
					Vec3 d = Normalize(FaceDirection(face, u, v));
					float basis[9];
					ShPolynomials(d, basis);
					const float *texel = faces + ((size_t(face) * size + y) * size + x) * 4;
					for (int c = 0; c < 9; ++c)
						for (int channel = 0; channel < 3; ++channel)
							sums[c][channel] += double(SH_COSINE[c] * SH_BASE[c] * basis[c] * texel[channel] * weight);
				}
			}
		// 系数 = 4π / 总权重 × 投影；再乘一次基函数常数，运行时只算多项式部分
		float *out = static_cast<float *>(ioTask.AddOutput(9 * 3 * sizeof(float)));
		for (int c = 0; c < 9; ++c)
			for (int channel = 0; channel < 3; ++channel)
				out[c * 3 + channel] = float(4 * double(PI) / weightSum * sums[c][channel]) * SH_BASE[c];
		return true;
	});
}
