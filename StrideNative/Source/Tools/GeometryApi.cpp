// Geometry processing for the pipeline: tangents, meshopt optimization and encoding, lightmap UVs,
// convex decomposition. The libraries run single-threaded inside a task; parallelism comes from
// processing many meshes as separate tasks on the shared job system.
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <VHACD.h>
#include <meshoptimizer.h>
#include <mikktspace.h>
#include <xatlas.h>

#include <cstring>
#include <memory>
#include <vector>

namespace
{
	using namespace StrideNative;

	struct TangentInput
	{
		const float *		mPositions;
		const float *		mNormals;
		const float *		mUvs;
		const uint32_t *	mIndices;
		uint32_t			mIndexCount;
		float *				mTangents;
	};

	uint32_t Corner(const SMikkTSpaceContext *inContext, int inFace, int inVertex)
	{
		return uint32_t(inFace) * 3 + uint32_t(inVertex);
	}

	const TangentInput &Input(const SMikkTSpaceContext *inContext)
	{
		return *static_cast<const TangentInput *>(inContext->m_pUserData);
	}

	int GetNumFaces(const SMikkTSpaceContext *inContext) { return int(Input(inContext).mIndexCount / 3); }
	int GetNumVerticesOfFace(const SMikkTSpaceContext *, const int) { return 3; }

	void GetPosition(const SMikkTSpaceContext *inContext, float outPosition[], const int inFace, const int inVertex)
	{
		const TangentInput &input = Input(inContext);
		std::memcpy(outPosition, input.mPositions + input.mIndices[Corner(inContext, inFace, inVertex)] * 3, 3 * sizeof(float));
	}

	void GetNormal(const SMikkTSpaceContext *inContext, float outNormal[], const int inFace, const int inVertex)
	{
		const TangentInput &input = Input(inContext);
		std::memcpy(outNormal, input.mNormals + input.mIndices[Corner(inContext, inFace, inVertex)] * 3, 3 * sizeof(float));
	}

	void GetTexCoord(const SMikkTSpaceContext *inContext, float outUv[], const int inFace, const int inVertex)
	{
		const TangentInput &input = Input(inContext);
		std::memcpy(outUv, input.mUvs + input.mIndices[Corner(inContext, inFace, inVertex)] * 2, 2 * sizeof(float));
	}

	void SetTangent(const SMikkTSpaceContext *inContext, const float inTangent[], const float inSign, const int inFace, const int inVertex)
	{
		float *tangent = Input(inContext).mTangents + Corner(inContext, inFace, inVertex) * 4;
		tangent[0] = inTangent[0];
		tangent[1] = inTangent[1];
		tangent[2] = inTangent[2];
		tangent[3] = inSign;
	}
}

SN_API int32_t st_generate_tangents(const float *positions, const float *normals, const float *uvs, const uint32_t *indices, uint32_t index_count, float *out_tangents4)
{
	if (positions == nullptr || normals == nullptr || uvs == nullptr || indices == nullptr || out_tangents4 == nullptr || index_count % 3 != 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "positions, normals, uvs, indices and output are required; index_count must be a multiple of 3");

	TangentInput input { positions, normals, uvs, indices, index_count, out_tangents4 };
	SMikkTSpaceInterface callbacks {};
	callbacks.m_getNumFaces = GetNumFaces;
	callbacks.m_getNumVerticesOfFace = GetNumVerticesOfFace;
	callbacks.m_getPosition = GetPosition;
	callbacks.m_getNormal = GetNormal;
	callbacks.m_getTexCoord = GetTexCoord;
	callbacks.m_setTSpaceBasic = SetTangent;
	SMikkTSpaceContext context { &callbacks, &input };
	return genTangSpaceDefault(&context) ? SN_OK : SetLastError(SN_ERROR_FAILED, "MikkTSpace failed");
}

SN_API int32_t st_meshopt_optimize(uint32_t *indices, uint32_t index_count, const float *positions, uint32_t vertex_count, uint32_t position_stride)
{
	if (indices == nullptr || positions == nullptr || index_count % 3 != 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "indices and positions are required; index_count must be a multiple of 3");

	meshopt_optimizeVertexCache(indices, indices, index_count, vertex_count);
	// 1.05: accept up to 5% worse vertex cache efficiency for less overdraw, meshoptimizer's suggestion.
	meshopt_optimizeOverdraw(indices, indices, index_count, positions, vertex_count, position_stride, 1.05f);
	return SN_OK;
}

SN_API int32_t st_meshopt_simplify(uint32_t *out_indices, const uint32_t *indices, uint32_t index_count, const float *positions, uint32_t vertex_count, uint32_t position_stride, uint32_t target_index_count, float target_error, float *out_error)
{
	if (out_indices == nullptr || indices == nullptr || positions == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "indices, positions and output are required");

	float error = 0.0f;
	size_t count = meshopt_simplify(out_indices, indices, index_count, positions, vertex_count, position_stride, target_index_count, target_error, 0, &error);
	if (out_error != nullptr)
		*out_error = error;
	return int32_t(count);
}

SN_API uint32_t st_meshopt_encode_vertex_bound(uint32_t vertex_count, uint32_t vertex_size)
{
	return uint32_t(meshopt_encodeVertexBufferBound(vertex_count, vertex_size));
}

SN_API uint32_t st_meshopt_encode_index_bound(uint32_t index_count, uint32_t vertex_count)
{
	return uint32_t(meshopt_encodeIndexBufferBound(index_count, vertex_count));
}

SN_API uint32_t st_meshopt_encode_vertices(void *dst, uint32_t dst_capacity, const void *vertices, uint32_t vertex_count, uint32_t vertex_size)
{
	return uint32_t(meshopt_encodeVertexBuffer(static_cast<unsigned char *>(dst), dst_capacity, vertices, vertex_count, vertex_size));
}

SN_API uint32_t st_meshopt_encode_indices(void *dst, uint32_t dst_capacity, const uint32_t *indices, uint32_t index_count)
{
	return uint32_t(meshopt_encodeIndexBuffer(static_cast<unsigned char *>(dst), dst_capacity, indices, index_count));
}

SN_API int32_t st_uv_unwrap(const float *positions, const float *normals, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count, float texels_per_unit)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (positions == nullptr || indices == nullptr || index_count % 3 != 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "positions and indices are required; index_count must be a multiple of 3");

	return RunTask("UvUnwrap", [=](Task &ioTask)
	{
		std::unique_ptr<xatlas::Atlas, decltype(&xatlas::Destroy)> atlas(xatlas::Create(), &xatlas::Destroy);
		xatlas::MeshDecl mesh;
		mesh.vertexPositionData = positions;
		mesh.vertexPositionStride = 3 * sizeof(float);
		mesh.vertexNormalData = normals;
		mesh.vertexNormalStride = normals != nullptr ? 3 * sizeof(float) : 0;
		mesh.vertexCount = vertex_count;
		mesh.indexData = indices;
		mesh.indexCount = index_count;
		mesh.indexFormat = xatlas::IndexFormat::UInt32;
		xatlas::AddMeshError error = xatlas::AddMesh(atlas.get(), mesh, 1);
		if (error != xatlas::AddMeshError::Success)
		{
			ioTask.Fail(std::string("xatlas: ") + xatlas::StringForEnum(error));
			return false;
		}

		xatlas::PackOptions pack;
		pack.texelsPerUnit = texels_per_unit;
		pack.padding = 2;
		xatlas::Generate(atlas.get(), xatlas::ChartOptions(), pack);
		if (atlas->meshCount != 1 || atlas->width == 0 || atlas->height == 0)
		{
			ioTask.Fail("xatlas produced no atlas");
			return false;
		}

		const xatlas::Mesh &result = atlas->meshes[0];
		uint32_t *xref = static_cast<uint32_t *>(ioTask.AddOutput(result.vertexCount * sizeof(uint32_t)));
		float *uv = static_cast<float *>(ioTask.AddOutput(result.vertexCount * 2 * sizeof(float)));
		for (uint32_t v = 0; v < result.vertexCount; ++v)
		{
			xref[v] = result.vertexArray[v].xref;
			uv[v * 2 + 0] = result.vertexArray[v].uv[0] / float(atlas->width);
			uv[v * 2 + 1] = result.vertexArray[v].uv[1] / float(atlas->height);
		}
		std::memcpy(ioTask.AddOutput(result.indexCount * sizeof(uint32_t)), result.indexArray, result.indexCount * sizeof(uint32_t));
		ioTask.SetResult(0, int32_t(atlas->width));
		ioTask.SetResult(1, int32_t(atlas->height));
		ioTask.SetResult(2, int32_t(result.vertexCount));
		return true;
	});
}

SN_API int32_t st_convex_decompose(const float *positions, uint32_t vertex_count, const uint32_t *indices, uint32_t index_count, uint32_t max_hulls, uint32_t voxel_resolution, uint32_t max_vertices_per_hull)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (positions == nullptr || indices == nullptr || index_count % 3 != 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "positions and indices are required; index_count must be a multiple of 3");

	return RunTask("ConvexDecompose", [=](Task &ioTask)
	{
		VHACD::IVHACD::Parameters params;
		params.m_maxConvexHulls = max_hulls;
		params.m_resolution = voxel_resolution;
		params.m_maxNumVerticesPerCH = max_vertices_per_hull;
		// V-HACD would start its own threads; the task already runs on a pool thread.
		params.m_asyncACD = false;

		VHACD::IVHACD *vhacd = VHACD::CreateVHACD();
		bool computed = vhacd->Compute(positions, vertex_count, indices, index_count / 3, params);
		uint32_t hull_count = computed ? vhacd->GetNConvexHulls() : 0;

		std::vector<int32_t> counts;
		std::vector<float> points;
		std::vector<uint32_t> triangles;
		for (uint32_t h = 0; h < hull_count; ++h)
		{
			VHACD::IVHACD::ConvexHull hull;
			vhacd->GetConvexHull(h, hull);
			counts.push_back(int32_t(hull.m_points.size()));
			counts.push_back(int32_t(hull.m_triangles.size()));
			for (const VHACD::Vertex &p : hull.m_points)
				points.insert(points.end(), { float(p.mX), float(p.mY), float(p.mZ) });
			for (const VHACD::Triangle &t : hull.m_triangles)
				triangles.insert(triangles.end(), { t.mI0, t.mI1, t.mI2 });
		}
		vhacd->Release();
		if (!computed)
		{
			ioTask.Fail("V-HACD failed");
			return false;
		}

		auto add = [&ioTask](const auto &inValues)
		{
			uint32_t size = uint32_t(inValues.size() * sizeof(inValues[0]));
			if (size > 0)
				std::memcpy(ioTask.AddOutput(size), inValues.data(), size);
			else
				ioTask.AddOutput(0);
		};
		add(counts);
		add(points);
		add(triangles);
		ioTask.SetResult(0, int32_t(hull_count));
		return true;
	});
}
