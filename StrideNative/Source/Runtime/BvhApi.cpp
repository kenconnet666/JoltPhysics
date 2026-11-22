// 光追用的 BVH（Runtime.h 的 sn_bvh_*，stride-ts 路线图 R2）：按表面积启发式（SAH）在三个轴上各分 16 个桶找最便宜的切法，
// 二叉树，左右孩子挨着放。三角形的版本另输出按叶子排好的三角形（v0、两条边），GPU 上直接做 Möller–Trumbore；包围盒的版本给
// 实例的顶层 BVH 用。节点布局见 Runtime.h。
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Runtime.h>

#include <algorithm>
#include <cfloat>
#include <cstring>
#include <string>
#include <vector>

namespace StrideNative
{
	namespace
	{
		constexpr int cBins = 16;
		/// SAH 的代价：走一个节点 1，测一个图元 1（GPU 上测三角形和测盒子差不多贵）。
		constexpr float cTraversalCost = 1.0f;

		struct Box
		{
			float mMin[3] = { FLT_MAX, FLT_MAX, FLT_MAX };
			float mMax[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

			void Grow(const Box &inOther)
			{
				for (int c = 0; c < 3; ++c)
				{
					mMin[c] = std::min(mMin[c], inOther.mMin[c]);
					mMax[c] = std::max(mMax[c], inOther.mMax[c]);
				}
			}

			void GrowPoint(const float *inPoint)
			{
				for (int c = 0; c < 3; ++c)
				{
					mMin[c] = std::min(mMin[c], inPoint[c]);
					mMax[c] = std::max(mMax[c], inPoint[c]);
				}
			}

			float Area() const
			{
				float dx = mMax[0] - mMin[0], dy = mMax[1] - mMin[1], dz = mMax[2] - mMin[2];
				if (dx < 0.0f)
					return 0.0f;
				return 2.0f * (dx * dy + dy * dz + dz * dx);
			}
		};

		struct Node
		{
			float		mMin[3];
			uint32_t	mFirst;
			float		mMax[3];
			uint32_t	mCount;
		};
		static_assert(sizeof(Node) == 32);

		/// 按图元的包围盒建树：返回节点，ioOrder 排成叶子的顺序。
		std::vector<Node> Build(const std::vector<Box> &inBoxes, std::vector<uint32_t> &ioOrder, uint32_t inMaxLeaf)
		{
			const uint32_t count = uint32_t(inBoxes.size());
			std::vector<float> centroids(size_t(count) * 3);
			for (uint32_t i = 0; i < count; ++i)
				for (int c = 0; c < 3; ++c)
					centroids[size_t(i) * 3 + c] = 0.5f * (inBoxes[i].mMin[c] + inBoxes[i].mMax[c]);
			ioOrder.resize(count);
			for (uint32_t i = 0; i < count; ++i)
				ioOrder[i] = i;

			std::vector<Node> nodes;
			nodes.reserve(std::max<size_t>(1, size_t(count) * 2));
			nodes.push_back({});
			struct Work { uint32_t mNode, mBegin, mEnd; };
			std::vector<Work> stack { { 0, 0, count } };
			while (!stack.empty())
			{
				Work work = stack.back();
				stack.pop_back();
				Box bounds, centroidBounds;
				for (uint32_t i = work.mBegin; i < work.mEnd; ++i)
				{
					bounds.Grow(inBoxes[ioOrder[i]]);
					centroidBounds.GrowPoint(&centroids[size_t(ioOrder[i]) * 3]);
				}
				const uint32_t n = work.mEnd - work.mBegin;
				auto makeLeaf = [&]
				{
					Node &node = nodes[work.mNode];
					std::memcpy(node.mMin, bounds.mMin, sizeof(node.mMin));
					std::memcpy(node.mMax, bounds.mMax, sizeof(node.mMax));
					node.mFirst = work.mBegin;
					node.mCount = n;
				};
				if (n <= inMaxLeaf && n <= 1)
				{
					makeLeaf();
					continue;
				}

				// 三个轴各 16 个桶，前后缀扫一遍找代价最小的切法
				float bestCost = FLT_MAX;
				int bestAxis = -1, bestSplit = 0;
				for (int axis = 0; axis < 3; ++axis)
				{
					float lo = centroidBounds.mMin[axis], hi = centroidBounds.mMax[axis];
					if (!(hi > lo))
						continue;
					float scale = cBins / (hi - lo);
					Box binBoxes[cBins];
					uint32_t binCounts[cBins] = {};
					for (uint32_t i = work.mBegin; i < work.mEnd; ++i)
					{
						int bin = std::min(cBins - 1, int((centroids[size_t(ioOrder[i]) * 3 + axis] - lo) * scale));
						binBoxes[bin].Grow(inBoxes[ioOrder[i]]);
						++binCounts[bin];
					}
					float rightArea[cBins];
					uint32_t rightCount[cBins];
					Box right;
					uint32_t rightN = 0;
					for (int b = cBins - 1; b > 0; --b)
					{
						right.Grow(binBoxes[b]);
						rightN += binCounts[b];
						rightArea[b] = right.Area();
						rightCount[b] = rightN;
					}
					Box left;
					uint32_t leftN = 0;
					for (int b = 0; b < cBins - 1; ++b)
					{
						left.Grow(binBoxes[b]);
						leftN += binCounts[b];
						if (leftN == 0 || rightCount[b + 1] == 0)
							continue;
						float cost = left.Area() * leftN + rightArea[b + 1] * rightCount[b + 1];
						if (cost < bestCost)
						{
							bestCost = cost;
							bestAxis = axis;
							bestSplit = b + 1;
						}
					}
				}

				float parentArea = std::max(bounds.Area(), 1e-20f);
				float leafCost = float(n);
				float splitCost = bestAxis < 0 ? FLT_MAX : cTraversalCost + bestCost / parentArea;
				uint32_t middle;
				if (bestAxis < 0)
				{
					// 质心都在一点上：分不开，小的就当叶子，大的按下标对半分
					if (n <= inMaxLeaf)
					{
						makeLeaf();
						continue;
					}
					middle = work.mBegin + n / 2;
				}
				else
				{
					if (n <= inMaxLeaf && leafCost <= splitCost)
					{
						makeLeaf();
						continue;
					}
					float lo = centroidBounds.mMin[bestAxis], scale = cBins / (centroidBounds.mMax[bestAxis] - lo);
					auto it = std::partition(ioOrder.begin() + work.mBegin, ioOrder.begin() + work.mEnd, [&](uint32_t inPrim)
					{
						return std::min(cBins - 1, int((centroids[size_t(inPrim) * 3 + bestAxis] - lo) * scale)) < bestSplit;
					});
					middle = uint32_t(it - ioOrder.begin());
					if (middle == work.mBegin || middle == work.mEnd)
						middle = work.mBegin + n / 2;
				}

				uint32_t leftChild = uint32_t(nodes.size());
				nodes.push_back({});
				nodes.push_back({});
				Node &node = nodes[work.mNode];
				std::memcpy(node.mMin, bounds.mMin, sizeof(node.mMin));
				std::memcpy(node.mMax, bounds.mMax, sizeof(node.mMax));
				node.mFirst = leftChild;
				node.mCount = 0;
				stack.push_back({ leftChild + 1, middle, work.mEnd });
				stack.push_back({ leftChild, work.mBegin, middle });
			}
			if (count == 0)
			{
				// 空的：一个什么也不包的叶子
				Node &root = nodes[0];
				for (int c = 0; c < 3; ++c)
				{
					root.mMin[c] = 0.0f;
					root.mMax[c] = -1.0f;
				}
				root.mFirst = 0;
				root.mCount = 0;
			}
			return nodes;
		}
	}
}

using namespace StrideNative;

SN_API int32_t sn_bvh_build_triangles(const float *positions, uint32_t vertex_count, const uint32_t *indices, uint32_t triangle_count)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if ((vertex_count > 0 && positions == nullptr) || (triangle_count > 0 && indices == nullptr))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_bvh_build_triangles: positions and indices are required");
	return RunTask("sn_bvh_build_triangles", [positions, vertex_count, indices, triangle_count](Task &ioTask)
	{
		std::vector<Box> boxes(triangle_count);
		for (uint32_t t = 0; t < triangle_count; ++t)
			for (int k = 0; k < 3; ++k)
			{
				uint32_t v = indices[size_t(t) * 3 + k];
				if (v >= vertex_count)
				{
					ioTask.Fail("sn_bvh_build_triangles: triangle " + std::to_string(t) + " uses vertex " + std::to_string(v) + " of " + std::to_string(vertex_count));
					return false;
				}
				boxes[t].GrowPoint(&positions[size_t(v) * 3]);
			}
		std::vector<uint32_t> order;
		std::vector<Node> nodes = Build(boxes, order, 4);
		std::memcpy(ioTask.AddOutput(uint32_t(nodes.size() * sizeof(Node))), nodes.data(), nodes.size() * sizeof(Node));
		float *triangles = static_cast<float *>(ioTask.AddOutput(std::max<uint32_t>(1, triangle_count) * 12 * sizeof(float)));
		for (uint32_t i = 0; i < triangle_count; ++i)
		{
			uint32_t t = order[i];
			const float *p0 = &positions[size_t(indices[size_t(t) * 3]) * 3];
			const float *p1 = &positions[size_t(indices[size_t(t) * 3 + 1]) * 3];
			const float *p2 = &positions[size_t(indices[size_t(t) * 3 + 2]) * 3];
			float *out = &triangles[size_t(i) * 12];
			out[0] = p0[0]; out[1] = p0[1]; out[2] = p0[2];
			std::memcpy(&out[3], &t, sizeof(uint32_t));
			out[4] = p1[0] - p0[0]; out[5] = p1[1] - p0[1]; out[6] = p1[2] - p0[2]; out[7] = 0.0f;
			out[8] = p2[0] - p0[0]; out[9] = p2[1] - p0[1]; out[10] = p2[2] - p0[2]; out[11] = 0.0f;
		}
		ioTask.SetResult(0, int32_t(nodes.size()));
		return true;
	});
}

SN_API int32_t sn_bvh_build_boxes(const float *boxes, uint32_t count)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (count > 0 && boxes == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sn_bvh_build_boxes: boxes are required");
	return RunTask("sn_bvh_build_boxes", [boxes, count](Task &ioTask)
	{
		std::vector<Box> input(count);
		for (uint32_t i = 0; i < count; ++i)
			for (int c = 0; c < 3; ++c)
			{
				input[i].mMin[c] = boxes[size_t(i) * 6 + c];
				input[i].mMax[c] = boxes[size_t(i) * 6 + 3 + c];
			}
		std::vector<uint32_t> order;
		std::vector<Node> nodes = Build(input, order, 2);
		std::memcpy(ioTask.AddOutput(uint32_t(nodes.size() * sizeof(Node))), nodes.data(), nodes.size() * sizeof(Node));
		std::memcpy(ioTask.AddOutput(std::max<uint32_t>(1, count) * sizeof(uint32_t)), order.data(), order.size() * sizeof(uint32_t));
		ioTask.SetResult(0, int32_t(nodes.size()));
		return true;
	});
}
