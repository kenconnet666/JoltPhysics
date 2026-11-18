// 视锥剔除（sn_transforms_cull）：每个实例的世界包围盒和视图的平面比较，可见的槽位按批次紧凑地写出。
// 判定和量距离的公式照 Stride 的 VisibilityGroup（FrustumContainsBox、CalculateMinMaxDistance）。
#include <Core/Core.h>
#include <Runtime/Transforms.h>
#include <StrideNative/Runtime.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>

using namespace JPH;
using namespace StrideNative;

namespace
{
	// 实例数乘视图数到这么多才分到多个线程上；再少的话分发的开销比剔除本身还大
	constexpr uint32_t cParallelThreshold = 8192;

	struct WorldBox
	{
		Vec3 mCenter;
		Vec3 mExtent;
	};

	/// 包围盒按世界矩阵变换：中心按点变换，半长按矩阵 3x3 部分各分量的绝对值变换。
	WorldBox TransformBox(const TransformStore &inStore, int32_t inSlot, Vec3Arg inCenter, Vec3Arg inExtent)
	{
		Mat44 world = Mat44::sLoadFloat4x4(reinterpret_cast<const Float4 *>(&inStore.mWorld[size_t(inSlot) * 16]));
		Vec3 extent = world.GetAxisX().Abs() * inExtent.GetX() + world.GetAxisY().Abs() * inExtent.GetY() + world.GetAxisZ().Abs() * inExtent.GetZ();
		return { world * inCenter, extent };
	}

	bool InsideFrustum(const sn_cull_view &inView, const WorldBox &inBox)
	{
		for (uint32_t i = 0; i < inView.plane_count; ++i)
		{
			Vec3 normal(inView.planes[i][0], inView.planes[i][1], inView.planes[i][2]);
			if (normal.Dot(inBox.mCenter) + normal.Abs().Dot(inBox.mExtent) <= -inView.planes[i][3])
				return false;
		}
		return true;
	}

	/// 一段批次的剔除结果，和这段里可见物体的距离范围（每个视图一对）。
	struct Range
	{
		float mMin[8];
		float mMax[8];
	};

	void CullBatches(const TransformStore &inStore, const int32_t *inSlots, uint32_t inSlotCount, const uint32_t *inBatchEnds, const float *inBounds, uint32_t inBatchCount,
		const sn_cull_view *inViews, uint32_t inViewCount, uint32_t inBatchBegin, uint32_t inBatchEnd, int32_t *outSlots, uint32_t *outCounts, Range &ioRange)
	{
		for (uint32_t b = inBatchBegin; b < inBatchEnd; ++b)
		{
			uint32_t begin = b == 0 ? 0 : inBatchEnds[b - 1];
			uint32_t end = inBatchEnds[b];
			Vec3 center(inBounds[b * 6], inBounds[b * 6 + 1], inBounds[b * 6 + 2]);
			Vec3 extent(inBounds[b * 6 + 3], inBounds[b * 6 + 4], inBounds[b * 6 + 5]);
			bool always_visible = extent.IsNearZero(0.0f);
			uint32_t counts[8] = {};
			for (uint32_t i = begin; i < end; ++i)
			{
				int32_t slot = inSlots[i];
				WorldBox box = always_visible ? WorldBox { Vec3::sZero(), Vec3::sZero() } : TransformBox(inStore, slot, center, extent);
				for (uint32_t v = 0; v < inViewCount; ++v)
				{
					const sn_cull_view &view = inViews[v];
					if (!always_visible && !InsideFrustum(view, box))
						continue;
					outSlots[size_t(v) * inSlotCount + begin + counts[v]++] = slot;
					if (always_visible || view.measure_distance == 0)
						continue;
					// 离平面最近和最远的两个角：按法线各分量的符号在最小、最大角之间挑
					Vec3 normal(view.distance_plane[0], view.distance_plane[1], view.distance_plane[2]);
					UVec4 negative = Vec3::sLess(normal, Vec3::sZero());
					Vec3 low = box.mCenter - box.mExtent, high = box.mCenter + box.mExtent;
					Vec3 near_corner = Vec3::sSelect(low, high, negative);
					Vec3 far_corner = Vec3::sSelect(high, low, negative);
					ioRange.mMin[v] = std::min(ioRange.mMin[v], normal.Dot(near_corner) - view.distance_plane[3]);
					ioRange.mMax[v] = std::max(ioRange.mMax[v], normal.Dot(far_corner) - view.distance_plane[3]);
				}
			}
			for (uint32_t v = 0; v < inViewCount; ++v)
				outCounts[size_t(v) * inBatchCount + b] = counts[v];
		}
	}
}

SN_API int32_t sn_transforms_cull(int32_t transforms, const int32_t *slots, uint32_t slot_count, const uint32_t *batch_ends, const float *bounds, uint32_t batch_count, sn_cull_view *views, uint32_t view_count, int32_t *out_slots, uint32_t *out_counts)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	TransformStore *store = LookupTransformStore(transforms);
	if (store == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (view_count > 8)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_transforms_cull: at most 8 views per call, got " + std::to_string(view_count));
	if (batch_count > 0 && batch_ends[batch_count - 1] != slot_count)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_transforms_cull: the last batch must end at slot_count");
	for (uint32_t i = 0; i < slot_count; ++i)
		if (!store->IsAlive(slots[i]))
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_transforms_cull: slot " + std::to_string(slots[i]) + " is not allocated");
	for (uint32_t v = 0; v < view_count; ++v)
		if (views[v].plane_count > 6)
			return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_transforms_cull: a view has at most 6 planes");

	Range total;
	std::fill(std::begin(total.mMin), std::end(total.mMin), std::numeric_limits<float>::infinity());
	std::fill(std::begin(total.mMax), std::end(total.mMax), -std::numeric_limits<float>::infinity());
	if (uint64_t(slot_count) * view_count < cParallelThreshold)
	{
		CullBatches(*store, slots, slot_count, batch_ends, bounds, batch_count, views, view_count, 0, batch_count, out_slots, out_counts, total);
	}
	else
	{
		// 各批写自己的区段，互不重叠；距离范围每段各算一份，最后合并
		std::mutex merge;
		ParallelFor(batch_count, [&](uint32_t inBegin, uint32_t inEnd)
		{
			Range range = total;
			CullBatches(*store, slots, slot_count, batch_ends, bounds, batch_count, views, view_count, inBegin, inEnd, out_slots, out_counts, range);
			std::lock_guard lock(merge);
			for (uint32_t v = 0; v < view_count; ++v)
			{
				total.mMin[v] = std::min(total.mMin[v], range.mMin[v]);
				total.mMax[v] = std::max(total.mMax[v], range.mMax[v]);
			}
		});
	}
	for (uint32_t v = 0; v < view_count; ++v)
	{
		views[v].min_distance = total.mMin[v];
		views[v].max_distance = total.mMax[v];
	}
	return SN_OK;
}
