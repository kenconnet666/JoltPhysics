// 矢量地形的求值器（Runtime.h 的 sn_terrain_*）：底是梯度噪声的分形（带解析导数，按导数压低陡处的细节），可选的冲沟，叠上矢量地物和
// 雕刻的笔画。
// 输入坐标是双精度：取整、求格子里的小数都在双精度里做，所以离原点几千公里的地方噪声照样连续、不起台阶。
#include <Core/Core.h>
#include <Core/HandleTable.h>
#include <Core/Tasks.h>
#include <StrideNative/Runtime.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace StrideNative
{
	namespace
	{
		struct Vec2d
		{
			double x, z;
		};

		/// 噪声值和它对 x、z 的导数。
		struct NoiseSample
		{
			float value, dx, dz;
		};

		uint32_t Hash(uint32_t inX, uint32_t inZ, uint32_t inSeed)
		{
			// murmur3 的收尾，三个输入先各乘一个奇数混开
			uint32_t h = inX * 0x8da6b343u ^ inZ * 0xd8163841u ^ inSeed * 0xcb1ab31fu;
			h ^= h >> 16;
			h *= 0x85ebca6bu;
			h ^= h >> 13;
			h *= 0xc2b2ae35u;
			h ^= h >> 16;
			return h;
		}

		/// 格点的梯度：两个 16 位映射到 [-1, 1]（不归一，免得用三角函数；各机器上逐位相同）。
		void Gradient(int64_t inX, int64_t inZ, uint32_t inSeed, float &outX, float &outZ)
		{
			uint32_t h = Hash(uint32_t(inX), uint32_t(inZ), inSeed);
			outX = float(h & 0xffffu) * (2.0f / 65535.0f) - 1.0f;
			outZ = float(h >> 16) * (2.0f / 65535.0f) - 1.0f;
		}

		/// 二维梯度噪声和解析导数（Inigo Quilez 的写法，五次的插值）。
		NoiseSample GradientNoise(double inX, double inZ, uint32_t inSeed)
		{
			double fx = std::floor(inX), fz = std::floor(inZ);
			int64_t ix = int64_t(fx), iz = int64_t(fz);
			float x = float(inX - fx), z = float(inZ - fz);
			float ux = x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f), uz = z * z * z * (z * (z * 6.0f - 15.0f) + 10.0f);
			float dux = 30.0f * x * x * (x * (x - 2.0f) + 1.0f), duz = 30.0f * z * z * (z * (z - 2.0f) + 1.0f);
			float gax, gaz, gbx, gbz, gcx, gcz, gdx, gdz;
			Gradient(ix, iz, inSeed, gax, gaz);
			Gradient(ix + 1, iz, inSeed, gbx, gbz);
			Gradient(ix, iz + 1, inSeed, gcx, gcz);
			Gradient(ix + 1, iz + 1, inSeed, gdx, gdz);
			float va = gax * x + gaz * z;
			float vb = gbx * (x - 1.0f) + gbz * z;
			float vc = gcx * x + gcz * (z - 1.0f);
			float vd = gdx * (x - 1.0f) + gdz * (z - 1.0f);
			float k = va - vb - vc + vd;
			NoiseSample s;
			s.value = va + ux * (vb - va) + uz * (vc - va) + ux * uz * k;
			s.dx = gax + ux * (gbx - gax) + uz * (gcx - gax) + ux * uz * (gax - gbx - gcx + gdx) + dux * (uz * k + (vb - va));
			s.dz = gaz + ux * (gbz - gaz) + uz * (gcz - gaz) + ux * uz * (gaz - gbz - gcz + gdz) + duz * (ux * k + (vc - va));
			return s;
		}

		float SmoothStep(float inEdge0, float inEdge1, float inX)
		{
			float t = std::clamp((inX - inEdge0) / (inEdge1 - inEdge0), 0.0f, 1.0f);
			return t * t * (3.0f - 2.0f * t);
		}

		/// 1 − (1 − t)²：从 0 出发是斜着上去的（不像 √t 那样竖着起步，坡度从 0 变大时不起折痕）。
		float EaseOut(float inT)
		{
			float t = 1.0f - std::clamp(inT, 0.0f, 1.0f);
			return 1.0f - t * t;
		}

		/// 1 − (1 − t)^p
		float PowInv(float inT, float inP)
		{
			return 1.0f - std::pow(1.0f - std::clamp(inT, 0.0f, 1.0f), inP);
		}

		constexpr float TWO_PI = 6.28318530717958647692f;

		/**
		 * 一层方向条纹（Phacelle 噪声的做法：Rune Skovbo Johansen 2026，自己的实现）：每个格子里一个随机位置的"斑"，斑上是一列平面波，
		 * 波沿 (inPX, inPZ) 变化（条纹顺着它的垂线走）。周围 4 × 4 个格子按距离加权（(1 − d² / R²)³，R = 1.5 格：钟形、到截断处
		 * 正好是 0，不出格子的缝；比指数衰减省掉 exp），加权平均出 cos、sin，再"半归一化"：长度平滑地拉向 1（波互相抵消的地方不会被硬拉成尖刺）。
		 * 输入是格子坐标（双精度），所有斑用同一个方向（这一点的方向）。
		 */
		void Stripes(double inQX, double inQZ, float inPX, float inPZ, uint32_t inSeed, float &outCos, float &outSin)
		{
			// 4 × 4 的窗口让 q 落在中间两格：窗口外的斑离 q 至少 1.5 格，权重已经是 0
			double bx = std::floor(inQX - 0.5), bz = std::floor(inQZ - 0.5);
			int64_t ix = int64_t(bx), iz = int64_t(bz);
			float sumC = 0.0f, sumS = 0.0f, sumW = 0.0f;
			for (int j = -1; j <= 2; ++j)
				for (int i = -1; i <= 2; ++i)
				{
					uint32_t h = Hash(uint32_t(ix + i), uint32_t(iz + j), inSeed);
					// 斑到 q 的位置差：先在双精度里减掉整数，再加格子里的随机偏移
					float rx = float(bx + i - inQX) + float(h & 0xffffu) * (1.0f / 65536.0f);
					float rz = float(bz + j - inQZ) + float(h >> 16) * (1.0f / 65536.0f);
					float d2 = (rx * rx + rz * rz) * (1.0f / 2.25f);
					if (d2 >= 1.0f)
						continue;
					float k = 1.0f - d2;
					float w = k * k * k;
					float phase = -TWO_PI * (rx * inPX + rz * inPZ);
					sumC += w * std::cos(phase);
					sumS += w * std::sin(phase);
					sumW += w;
				}
			if (sumW <= 0.0f)
			{
				outCos = 1.0f;
				outSin = 0.0f;
				return;
			}
			float c = sumC / sumW, s = sumS / sumW;
			// 平滑地饱和：2L / √(1 + 4L²)（短的放大到接近两倍、长的趋近 1）；按 min(1, 2L) 截断的话截断处有折痕，地形的网格采样不到这么
			// 细的棱，远处出锯齿
			float length2 = c * c + s * s;
			float scale = 2.0f / std::sqrt(1.0f + 4.0f * length2);
			outCos = c * scale;
			outSin = s * scale;
		}

		struct Feature
		{
			sn_terrain_feature desc;
			std::vector<Vec2d> points;
			double minX, minZ, maxX, maxZ;	///< 包围盒加上过渡宽度，盒外不用算距离
			/// 折线：每个点的高度；按格子分好的线段（一个格子里列着够得着它的线段，长路不用每个点都比所有线段）
			std::vector<float> heights;
			double cellSize = 0.0;
			std::unordered_map<uint64_t, std::vector<uint32_t>> cells;
		};

		uint64_t CellKey(int64_t inX, int64_t inZ)
		{
			return (uint64_t(uint32_t(inX)) << 32) | uint32_t(inZ);
		}

		struct Stroke
		{
			sn_terrain_stroke desc;
		};

		/// 一份地形的描述；创建后不变，任务拷 shared_ptr 用。
		struct TerrainData
		{
			sn_terrain_noise noise;
			std::vector<Feature> features;
			std::vector<Stroke> strokes;

			/// 底：分形噪声（可选的域扭曲、山脊、按导数的侵蚀），octaves 层。
			float Base(double inX, double inZ, uint32_t inOctaves) const
			{
				const double scale = noise.scale;
				double px = inX / scale, pz = inZ / scale;
				if (noise.warp != 0.0f)
				{
					// 用两路低频噪声挪采样点（米换成噪声空间）
					double w = noise.warp / scale;
					NoiseSample a = GradientNoise(px * 0.5 + 17.3, pz * 0.5 + 41.9, noise.seed ^ 0x51ed27u);
					NoiseSample b = GradientNoise(px * 0.5 - 23.1, pz * 0.5 + 7.7, noise.seed ^ 0x2c1b3au);
					px += w * a.value;
					pz += w * b.value;
				}
				float sum = 0.0f, amplitude = 1.0f, norm = 0.0f, ddx = 0.0f, ddz = 0.0f;
				for (uint32_t octave = 0; octave < inOctaves; ++octave)
				{
					NoiseSample n = GradientNoise(px, pz, noise.seed + octave * 0x9e3779b9u);
					float value = n.value;
					if (noise.ridged > 0.0f)
					{
						float r = 1.0f - 2.0f * std::fabs(n.value);
						value += noise.ridged * ((r * r - 0.5f) - value);
					}
					ddx += n.dx;
					ddz += n.dz;
					sum += amplitude * value / (1.0f + noise.erosion * (ddx * ddx + ddz * ddz));
					norm += amplitude;
					amplitude *= noise.gain;
					// 每层转一个角度（同 IQ 的 m2），层与层的格子不对齐，看不出网格的走向
					double nx = 0.8 * px - 0.6 * pz, nz = 0.6 * px + 0.8 * pz;
					px = nx * noise.lacunarity;
					pz = nz * noise.lacunarity;
				}
				return noise.offset + noise.height * (norm > 0.0f ? sum / norm * 1.4f : 0.0f);
			}

			float Height(double inX, double inZ) const
			{
				float h = Gullies(inX, inZ, Base(inX, inZ, noise.octaves));
				for (const Feature &feature : features)
					h = feature.desc.type == SN_TERRAIN_FEATURE_PATH ? ApplyPath(feature, inX, inZ, h) : ApplyFeature(feature, inX, inZ, h);
				for (const Stroke &stroke : strokes)
					h = ApplyStroke(stroke.desc, inX, inZ, h);
				return h;
			}

			/**
			 * 冲沟（Rune Skovbo Johansen 2026 的 erosion filter 的思路，自己的实现，逐点算、不模拟，流式加载的块各算各的也接得上）：
			 * 每层一列顺着下坡方向的条纹（Stripes），cos 是这一层的起伏；下一层的方向加上这一层条纹的坡（按符号，像三角波），细的冲沟
			 * 就从粗的两侧斜着分出去。坡缓的地方（山顶、谷底、平地）冲沟没有方向：按遮罩淡向目标——谷底的高度以下是 −1（谷）、
			 * 山顶以上是 +1（尖），淡完的值又是下一层的目标；遮罩一层层乘上"这一层条纹的坡"，细的冲沟不压在粗的冲沟的脊、槽上。
			 * 方向取一半层数的底的坡（大的起伏定走向）。
			 */
			float Gullies(double inX, double inZ, float inHeight) const
			{
				const float strength = noise.gully_strength;
				if (!(strength > 0.0f) || noise.gully_octaves == 0)
					return inHeight;
				// 走向只看大的起伏：三层的底、隔四分之一个冲沟间距的中心差分——方向场要平滑，底上的棱（山脊噪声、扭曲）会让方向
				// 突然转个角度，条纹跟着断开，地上出一道道小坎
				const uint32_t octaves = std::min(3u, std::max(1u, noise.octaves));
				const double e = std::max(0.01, double(noise.gully_scale) * 0.25);
				float gx = float(double(Base(inX + e, inZ, octaves) - Base(inX - e, inZ, octaves)) / (2.0 * e));
				float gz = float(double(Base(inX, inZ + e, octaves) - Base(inX, inZ - e, octaves)) / (2.0 * e));
				float slope = std::sqrt(gx * gx + gz * gz);
				if (slope < 1e-6f)
				{
					gx = 1.0f;
					gz = 0.0f;
				}
				const float range = noise.gully_peak - noise.gully_valley;
				float fade = range > 0.0f ? std::clamp(2.0f * (inHeight - noise.gully_valley) / range - 1.0f, -1.0f, 1.0f) : 0.0f;
				// 坡 0.5（约 27°）以上全是冲沟
				float mask = EaseOut(slope * 2.0f);
				float total = 0.0f, norm = 0.0f, amplitude = 1.0f;
				double frequency = 1.0 / double(noise.gully_scale);
				float sx = gx, sz = gz;
				for (uint32_t octave = 0; octave < noise.gully_octaves; ++octave)
				{
					float length = std::sqrt(sx * sx + sz * sz);
					float dx = sx / length, dz = sz / length;
					// 条纹顺着下坡方向走：相位沿它的垂线变
					float px = -dz, pz = dx;
					float c, s;
					Stripes(inX * frequency, inZ * frequency, px, pz, noise.seed ^ (0x7f4a7c15u + octave * 0x2545f491u), c, s);
					float faded = fade + (c - fade) * mask;
					total += faded * amplitude;
					norm += amplitude;
					fade = faded;
					// sin² 而不是 |sin|：遮罩在条纹的脊、槽上平滑地到 0，不起折痕
					mask = PowInv(mask, noise.gully_detail) * EaseOut(s * s * 4.0f);
					// 这一层条纹的坡（米每米）加到走向上：用 sin 本身（连续的），按符号的话正负交界处下一层的方向跳一下，地上出坎
					float gullySlope = strength * amplitude * TWO_PI * float(frequency) * s;
					sx += px * gullySlope;
					sz += pz * gullySlope;
					amplitude *= 0.5f;
					frequency *= 2.0;
				}
				return inHeight + strength * total / norm;
			}

			static float ApplyFeature(const Feature &inFeature, double inX, double inZ, float inHeight)
			{
				if (inX < inFeature.minX || inX > inFeature.maxX || inZ < inFeature.minZ || inZ > inFeature.maxZ)
					return inHeight;
				// 到多边形的有符号距离（里面是负的）：逐边取最近距离，奇偶规则判里外
				const std::vector<Vec2d> &p = inFeature.points;
				double best = 1e300;
				bool inside = false;
				for (size_t i = 0, j = p.size() - 1; i < p.size(); j = i++)
				{
					double ex = p[i].x - p[j].x, ez = p[i].z - p[j].z;
					double wx = inX - p[j].x, wz = inZ - p[j].z;
					double len = ex * ex + ez * ez;
					double t = len > 0.0 ? std::clamp((wx * ex + wz * ez) / len, 0.0, 1.0) : 0.0;
					double dx = wx - ex * t, dz = wz - ez * t;
					best = std::min(best, dx * dx + dz * dz);
					if ((p[i].z > inZ) != (p[j].z > inZ) && inX < (p[j].x - p[i].x) * (inZ - p[i].z) / (p[j].z - p[i].z) + p[i].x)
						inside = !inside;
				}
				float distance = inside ? 0.0f : float(std::sqrt(best));
				const sn_terrain_feature &d = inFeature.desc;
				float weight = d.falloff > 0.0f ? 1.0f - SmoothStep(0.0f, d.falloff, distance) : (inside ? 1.0f : 0.0f);
				weight *= d.strength;
				// 完全压平时直接给目标高度（不经过插值的舍入），平台上的点逐位等于 height
				return weight >= 1.0f ? d.height : inHeight + (d.height - inHeight) * weight;
			}

			/// 折线：离它最近的线段上的高度（加偏移）是目标，宽度以内全压、往外 falloff 米内过渡；按模式只削、只填或都做。
			static float ApplyPath(const Feature &inPath, double inX, double inZ, float inHeight)
			{
				if (inX < inPath.minX || inX > inPath.maxX || inZ < inPath.minZ || inZ > inPath.maxZ)
					return inHeight;
				auto cell = inPath.cells.find(CellKey(int64_t(std::floor(inX / inPath.cellSize)), int64_t(std::floor(inZ / inPath.cellSize))));
				if (cell == inPath.cells.end())
					return inHeight;
				const sn_terrain_feature &d = inPath.desc;
				const double half = 0.5 * d.width, reach = half + std::max(0.0f, d.falloff);
				double best = reach * reach;
				float target = 0.0f;
				bool found = false;
				for (uint32_t s : cell->second)
				{
					const Vec2d &a = inPath.points[s], &b = inPath.points[s + 1];
					double ex = b.x - a.x, ez = b.z - a.z, wx = inX - a.x, wz = inZ - a.z;
					double len = ex * ex + ez * ez;
					double t = len > 0.0 ? std::clamp((wx * ex + wz * ez) / len, 0.0, 1.0) : 0.0;
					double dx = wx - ex * t, dz = wz - ez * t, d2 = dx * dx + dz * dz;
					if (d2 < best)
					{
						best = d2;
						target = inPath.heights[s] + float(t) * (inPath.heights[s + 1] - inPath.heights[s]);
						found = true;
					}
				}
				if (!found)
					return inHeight;
				float distance = float(std::max(0.0, std::sqrt(best) - half));
				float weight = d.falloff > 0.0f ? 1.0f - SmoothStep(0.0f, d.falloff, distance) : (distance <= 0.0f ? 1.0f : 0.0f);
				weight *= d.strength;
				target += d.height;
				float h = weight >= 1.0f ? target : inHeight + (target - inHeight) * weight;
				if (d.mode == SN_TERRAIN_PATH_CARVE)
					return std::min(inHeight, h);
				if (d.mode == SN_TERRAIN_PATH_FILL)
					return std::max(inHeight, h);
				return h;
			}

			float ApplyStroke(const sn_terrain_stroke &inStroke, double inX, double inZ, float inHeight) const
			{
				double dx = inX - inStroke.x, dz = inZ - inStroke.z;
				double r2 = (dx * dx + dz * dz) / (double(inStroke.radius) * inStroke.radius);
				if (r2 >= 1.0)
					return inHeight;
				// (1 - r²)²：中心最强、边上导数为零，叠很多笔也不起棱
				float falloff = float((1.0 - r2) * (1.0 - r2));
				switch (inStroke.op)
				{
				case SN_TERRAIN_STROKE_RAISE:
					return inHeight + inStroke.strength * falloff;
				case SN_TERRAIN_STROKE_LOWER:
					return inHeight - inStroke.strength * falloff;
				case SN_TERRAIN_STROKE_FLATTEN:
					return inHeight + (inStroke.height - inHeight) * std::clamp(inStroke.strength, 0.0f, 1.0f) * falloff;
				case SN_TERRAIN_STROKE_SMOOTH:
				{
					// 靠向少一半层数的底：去掉细节、留下大的起伏（地物和别的笔画不在目标里）
					float target = Base(inX, inZ, std::max(1u, noise.octaves / 2));
					return inHeight + (target - inHeight) * std::clamp(inStroke.strength, 0.0f, 1.0f) * falloff;
				}
				default:
					return inHeight;
				}
			}
		};

		/// 句柄表里存 shared_ptr：任务拷一份，跑着时销毁地形也安全。
		struct Terrain
		{
			std::shared_ptr<const TerrainData> data;
		};

		constexpr uint32_t MAX_PAGE_SAMPLES = 1u << 24;
	}
}

using namespace StrideNative;

SN_API int32_t sn_terrain_create(const sn_terrain_noise *noise, const sn_terrain_feature *features, uint32_t feature_count, const double *points, const float *point_heights, uint32_t point_count, const sn_terrain_stroke *strokes, uint32_t stroke_count)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (noise == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_create: noise is required");
	if (noise->octaves > 16)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_create: octaves must be in [0, 16], got " + std::to_string(noise->octaves));
	if (!(noise->scale > 0.0f) || !(noise->lacunarity > 0.0f) || noise->erosion < 0.0f)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_create: scale and lacunarity must be positive, erosion non-negative");
	if (noise->gully_strength > 0.0f && (noise->gully_octaves > 8 || !(noise->gully_scale > 0.0f) || !(noise->gully_detail > 0.0f)))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_create: gullies need octaves in [0, 8] and a positive scale and detail");
	if ((feature_count > 0 && features == nullptr) || (point_count > 0 && points == nullptr) || (stroke_count > 0 && strokes == nullptr))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_create: a non-empty array is null");

	auto data = std::make_shared<TerrainData>();
	data->noise = *noise;
	data->features.reserve(feature_count);
	for (uint32_t i = 0; i < feature_count; ++i)
	{
		const sn_terrain_feature &desc = features[i];
		const bool path = desc.type == SN_TERRAIN_FEATURE_PATH;
		if (desc.type != SN_TERRAIN_FEATURE_FLATTEN && !path)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_create: feature " + std::to_string(i) + " has unknown type " + std::to_string(desc.type));
		const uint32_t minPoints = path ? 2u : 3u;
		if (desc.point_count < minPoints || uint64_t(desc.first_point) + desc.point_count > point_count)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_create: feature " + std::to_string(i) + " needs at least " + std::to_string(minPoints) + " points inside the point array");
		if (path && (point_heights == nullptr || !(desc.width > 0.0f) || desc.mode > SN_TERRAIN_PATH_FILL))
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_create: path feature " + std::to_string(i) + " needs point heights, a positive width and a known mode");
		Feature feature{ desc, {}, 1e300, 1e300, -1e300, -1e300, {}, 0.0, {} };
		for (uint32_t p = 0; p < desc.point_count; ++p)
		{
			Vec2d point{ points[(desc.first_point + p) * 2], points[(desc.first_point + p) * 2 + 1] };
			feature.points.push_back(point);
			feature.minX = std::min(feature.minX, point.x);
			feature.minZ = std::min(feature.minZ, point.z);
			feature.maxX = std::max(feature.maxX, point.x);
			feature.maxZ = std::max(feature.maxZ, point.z);
		}
		double margin = std::max(0.0f, desc.falloff) + (path ? 0.5 * desc.width : 0.0);
		if (path)
		{
			// 线段按格子分：格子边长取够得着的距离（半宽加过渡，至少 1 米），每段放进它的包围盒（加够得着的距离）盖到的格子
			feature.heights.assign(point_heights + desc.first_point, point_heights + desc.first_point + desc.point_count);
			feature.cellSize = std::max(1.0, margin);
			for (uint32_t s = 0; s + 1 < desc.point_count; ++s)
			{
				const Vec2d &a = feature.points[s], &b = feature.points[s + 1];
				int64_t x0 = int64_t(std::floor((std::min(a.x, b.x) - margin) / feature.cellSize)), x1 = int64_t(std::floor((std::max(a.x, b.x) + margin) / feature.cellSize));
				int64_t z0 = int64_t(std::floor((std::min(a.z, b.z) - margin) / feature.cellSize)), z1 = int64_t(std::floor((std::max(a.z, b.z) + margin) / feature.cellSize));
				for (int64_t cz = z0; cz <= z1; ++cz)
					for (int64_t cx = x0; cx <= x1; ++cx)
						feature.cells[CellKey(cx, cz)].push_back(s);
			}
		}
		feature.minX -= margin;
		feature.minZ -= margin;
		feature.maxX += margin;
		feature.maxZ += margin;
		data->features.push_back(std::move(feature));
	}
	data->strokes.reserve(stroke_count);
	for (uint32_t i = 0; i < stroke_count; ++i)
	{
		if (strokes[i].op > SN_TERRAIN_STROKE_SMOOTH || !(strokes[i].radius > 0.0f))
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_create: stroke " + std::to_string(i) + " has an unknown op or a non-positive radius");
		data->strokes.push_back({ strokes[i] });
	}
	auto terrain = std::make_unique<Terrain>();
	terrain->data = std::move(data);
	return GetTable<Terrain>().Add(std::move(terrain));
}

SN_API void sn_terrain_destroy(int32_t terrain)
{
	GetTable<Terrain>().Remove(terrain);
}

SN_API float sn_terrain_height(int32_t terrain, double x, double z)
{
	Terrain *t = Lookup<Terrain>(terrain, "terrain");
	return t != nullptr ? t->data->Height(x, z) : 0.0f;
}

SN_API int32_t sn_terrain_heights(int32_t terrain, double x0, double z0, double spacing, uint32_t count_x, uint32_t count_z)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	Terrain *t = Lookup<Terrain>(terrain, "terrain");
	if (t == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (count_x == 0 || count_z == 0 || uint64_t(count_x) * count_z > MAX_PAGE_SAMPLES || !(spacing > 0.0))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_terrain_heights: need 1..2^24 samples and a positive spacing");
	std::shared_ptr<const TerrainData> data = t->data;
	return RunTask("sn_terrain_heights", [data, x0, z0, spacing, count_x, count_z](Task &ioTask)
	{
		float *out = static_cast<float *>(ioTask.AddOutput(count_x * count_z * sizeof(float)));
		for (uint32_t j = 0; j < count_z; ++j)
		{
			// 每个点由原点加整数倍的间距算（双精度），不逐个累加，页和页的接缝上的点逐位相同
			double z = z0 + spacing * j;
			for (uint32_t i = 0; i < count_x; ++i)
				out[j * count_x + i] = data->Height(x0 + spacing * i, z);
		}
		return true;
	});
}
