// Geometry processing for the pipeline: tangents, meshopt optimization and encoding, lightmap UVs,
// convex decomposition. The libraries run single-threaded inside a task; parallelism comes from
// processing many meshes as separate tasks on the shared job system.
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <VHACD.h>
#include <manifold/manifold.h>
#include <meshoptimizer.h>
#include <mikktspace.h>
#include <xatlas.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
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

namespace
{
	/// One brush of st_brush_csg, built and placed; Fail() on bad input.
	bool BuildBrush(const float *&ioCursor, const float *inEnd, uint32_t inIndex, manifold::Manifold &outShape, int &outOperation, std::string &outError)
	{
		auto need = [&](size_t inCount) { return size_t(inEnd - ioCursor) >= inCount; };
		if (!need(18))
		{
			outError = "brush " + std::to_string(inIndex) + ": the stream ends before its kind, operation and transform";
			return false;
		}
		const int kind = int(ioCursor[0]);
		outOperation = int(ioCursor[1]);
		const float *m = ioCursor + 2;
		ioCursor += 18;
		manifold::Manifold shape;
		switch (kind)
		{
		case ST_BRUSH_BOX:
			if (!need(3)) break;
			shape = manifold::Manifold::Cube(manifold::vec3(ioCursor[0], ioCursor[1], ioCursor[2]), true);
			ioCursor += 3;
			break;
		case ST_BRUSH_PRISM:
		{
			if (!need(2)) break;
			const double height = ioCursor[0];
			const uint32_t count = uint32_t(ioCursor[1]);
			ioCursor += 2;
			if (count < 3 || !need(size_t(count) * 2))
			{
				outError = "brush " + std::to_string(inIndex) + ": a prism needs at least 3 points";
				return false;
			}
			// Manifold extrudes an XY polygon along +Z; ours is an XZ polygon extruded along +Y. (x, z) -> (x, -z), then rotate -90 degrees
			// about X so Manifold's Z becomes our Y.
			manifold::SimplePolygon polygon(count);
			double area = 0;
			for (uint32_t i = 0; i < count; ++i)
				polygon[i] = manifold::vec2(ioCursor[i * 2], -ioCursor[i * 2 + 1]);
			for (uint32_t i = 0; i < count; ++i)
			{
				const manifold::vec2 &a = polygon[i], &b = polygon[(i + 1) % count];
				area += a.x * b.y - b.x * a.y;
			}
			if (area < 0)
				std::reverse(polygon.begin(), polygon.end());
			ioCursor += size_t(count) * 2;
			shape = manifold::Manifold::Extrude({ polygon }, height).Rotate(-90, 0, 0);
			break;
		}
		case ST_BRUSH_CYLINDER:
			if (!need(4)) break;
			shape = manifold::Manifold::Cylinder(ioCursor[0], ioCursor[1], ioCursor[2], int(ioCursor[3]), true).Rotate(-90, 0, 0);
			ioCursor += 4;
			break;
		case ST_BRUSH_SPHERE:
			if (!need(2)) break;
			shape = manifold::Manifold::Sphere(ioCursor[0], int(ioCursor[1]));
			ioCursor += 2;
			break;
		default:
			outError = "brush " + std::to_string(inIndex) + ": unknown kind " + std::to_string(kind);
			return false;
		}
		if (shape.Status() != manifold::Manifold::Error::NoError || shape.IsEmpty())
		{
			outError = "brush " + std::to_string(inIndex) + ": its parameters make no solid (kind " + std::to_string(kind) + ")";
			return false;
		}
		manifold::mat3x4 transform;
		for (int column = 0; column < 4; ++column)
			for (int row = 0; row < 3; ++row)
				transform[column][row] = m[column * 4 + row];
		outShape = shape.Transform(transform).AsOriginal();
		return true;
	}
}

SN_API int32_t st_brush_csg(const float *brushes, uint32_t float_count)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (brushes == nullptr && float_count > 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "brushes is null");

	return RunTask("BrushCsg", [=](Task &ioTask)
	{
		manifold::Manifold result;
		std::unordered_map<uint32_t, uint32_t> brush_of_original;
		const float *cursor = brushes, *end = brushes + float_count;
		for (uint32_t index = 0; cursor < end; ++index)
		{
			manifold::Manifold shape;
			int operation = ST_BRUSH_ADD;
			std::string error;
			if (!BuildBrush(cursor, end, index, shape, operation, error))
			{
				ioTask.Fail(error.empty() ? "brush " + std::to_string(index) + ": the stream ends inside its params" : error);
				return false;
			}
			brush_of_original[uint32_t(shape.OriginalID())] = index;
			switch (operation)
			{
			case ST_BRUSH_ADD: result += shape; break;
			case ST_BRUSH_SUBTRACT: result -= shape; break;
			case ST_BRUSH_INTERSECT: result ^= shape; break;
			default:
				ioTask.Fail("brush " + std::to_string(index) + ": unknown operation " + std::to_string(operation));
				return false;
			}
		}
		if (result.Status() != manifold::Manifold::Error::NoError)
		{
			ioTask.Fail("the brushes do not combine into a valid solid (Manifold error " + std::to_string(int(result.Status())) + ")");
			return false;
		}

		const manifold::MeshGL mesh = result.GetMeshGL();
		const uint32_t vertex_count = uint32_t(mesh.NumVert()), triangle_count = uint32_t(mesh.NumTri());
		float *positions = static_cast<float *>(ioTask.AddOutput(size_t(vertex_count) * 3 * sizeof(float)));
		for (uint32_t v = 0; v < vertex_count; ++v)
			for (int k = 0; k < 3; ++k)
				positions[v * 3 + k] = mesh.vertProperties[size_t(v) * mesh.numProp + k];
		std::memcpy(ioTask.AddOutput(size_t(triangle_count) * 3 * sizeof(uint32_t)), mesh.triVerts.data(), size_t(triangle_count) * 3 * sizeof(uint32_t));
		uint32_t *owner = static_cast<uint32_t *>(ioTask.AddOutput(size_t(triangle_count) * sizeof(uint32_t)));
		for (size_t run = 0; run < mesh.runOriginalID.size(); ++run)
		{
			auto found = brush_of_original.find(mesh.runOriginalID[run]);
			const uint32_t brush = found != brush_of_original.end() ? found->second : 0;
			for (uint32_t t = mesh.runIndex[run] / 3; t < mesh.runIndex[run + 1] / 3; ++t)
				owner[t] = brush;
		}
		ioTask.SetResult(0, int32_t(triangle_count));
		return true;
	});
}

namespace
{
	/// One shape of st_sdf_mesh: where it is (world -> local) and its distance function in local space.
	struct SdfShape
	{
		int kind = 0;
		int operation = ST_BRUSH_ADD;
		double smoothness = 0;
		manifold::mat3x4 toLocal;		///< world -> local (inverse of the given transform)
		double scale = 1;				///< distances in local space times this are world distances
		manifold::vec3 center;			///< world position of the local origin
		double params[4] = {};
		double bound = 0;				///< radius of a sphere around the local origin holding the shape (local units)
	};

	double Length2(double inX, double inY) { return std::sqrt(inX * inX + inY * inY); }

	/// Signed distance of a local point to the shape (negative inside), in local units.
	double LocalDistance(const SdfShape &inShape, const manifold::vec3 &p)
	{
		const double *a = inShape.params;
		switch (inShape.kind)
		{
		case ST_SDF_SPHERE:
			return manifold::la::length(p) - a[0];
		case ST_SDF_BOX:
		{
			const double r = std::min(a[3], std::min(a[0], std::min(a[1], a[2])));
			const manifold::vec3 q = manifold::la::abs(p) - manifold::vec3(a[0] - r, a[1] - r, a[2] - r);
			return manifold::la::length(manifold::la::max(q, manifold::vec3(0.0))) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.0) - r;
		}
		case ST_SDF_CAPSULE:
		{
			const double y = std::clamp(p.y, -a[0], a[0]);
			return manifold::la::length(manifold::vec3(p.x, p.y - y, p.z)) - a[1];
		}
		case ST_SDF_CYLINDER:
		{
			const double r = std::min(a[2], std::min(a[0], a[1]));
			const double dx = Length2(p.x, p.z) - (a[1] - r), dy = std::abs(p.y) - (a[0] - r);
			return std::min(std::max(dx, dy), 0.0) + Length2(std::max(dx, 0.0), std::max(dy, 0.0)) - r;
		}
		case ST_SDF_TORUS:
			return Length2(Length2(p.x, p.z) - a[0], p.y) - a[1];
		case ST_SDF_ELLIPSOID:
		{
			// Inigo Quilez's bound for ellipsoids: exact on the axes, close elsewhere
			const manifold::vec3 r(a[0], a[1], a[2]);
			const double k0 = manifold::la::length(p / r), k1 = manifold::la::length(p / (r * r));
			return k1 > 0 ? k0 * (k0 - 1.0) / k1 : -std::min(a[0], std::min(a[1], a[2]));
		}
		case ST_SDF_CONE:
		{
			// Inigo Quilez's exact capped cone: bottom radius a[1] at y = -h, top radius a[2] at y = +h
			const double h = a[0], r1 = a[1], r2 = a[2];
			const double qx = Length2(p.x, p.z), qy = p.y;
			const double k2x = r2 - r1, k2y = 2.0 * h;
			const double cax = qx - std::min(qx, qy < 0.0 ? r1 : r2), cay = std::abs(qy) - h;
			const double t = std::clamp(((r2 - qx) * k2x + (h - qy) * k2y) / (k2x * k2x + k2y * k2y), 0.0, 1.0);
			const double cbx = qx - r2 + k2x * t, cby = qy - h + k2y * t;
			const double s = (cbx < 0.0 && cay < 0.0) ? -1.0 : 1.0;
			return s * std::sqrt(std::min(cax * cax + cay * cay, cbx * cbx + cby * cby));
		}
		case ST_SDF_ROUND_CONE:
		{
			// Inigo Quilez's round cone: spheres of radius a[1] at y = -a[0]/2 and a[2] at y = +a[0]/2, joined by their tangent cone
			const double h = a[0], r1 = a[1], r2 = a[2];
			const double b = (r1 - r2) / h, c = std::sqrt(std::max(0.0, 1.0 - b * b));
			const double qx = Length2(p.x, p.z), qy = p.y + h * 0.5;
			const double k = -b * qx + c * qy;
			if (k < 0.0)
				return Length2(qx, qy) - r1;
			if (k > c * h)
				return Length2(qx, qy - h) - r2;
			return qx * c + qy * b - r1;
		}
		case ST_SDF_HEX_PRISM:
		{
			// Inigo Quilez's hexagonal prism with the hexagon in XZ (a[1] is the apothem) and the height along Y
			const double kx = -0.8660254037844386, ky = 0.5, kz = 0.5773502691896258;
			double x = std::abs(p.x), z = std::abs(p.z);
			const double fold = 2.0 * std::min(kx * x + ky * z, 0.0);
			x -= fold * kx;
			z -= fold * ky;
			const double edge = std::clamp(x, -kz * a[1], kz * a[1]);
			const double dx = Length2(x - edge, z - a[1]) * (z - a[1] < 0.0 ? -1.0 : 1.0), dy = std::abs(p.y) - a[0];
			return std::min(std::max(dx, dy), 0.0) + Length2(std::max(dx, 0.0), std::max(dy, 0.0));
		}
		}
		return 1e9;
	}

	double ShapeDistance(const SdfShape &inShape, const manifold::vec3 &inWorld)
	{
		return LocalDistance(inShape, inShape.toLocal * manifold::vec4(inWorld, 1.0)) * inShape.scale;
	}

	/// Polynomial smooth minimum (Inigo Quilez): blends over a band of width k.
	double SmoothMin(double inA, double inB, double inK)
	{
		if (inK <= 0)
			return std::min(inA, inB);
		const double h = std::max(inK - std::abs(inA - inB), 0.0) / inK;
		return std::min(inA, inB) - h * h * inK * 0.25;
	}

	double SceneDistance(const std::vector<SdfShape> &inShapes, const manifold::vec3 &inWorld)
	{
		double d = 1e9;
		for (const SdfShape &shape : inShapes)
		{
			const double s = ShapeDistance(shape, inWorld);
			switch (shape.operation)
			{
			case ST_BRUSH_ADD: d = SmoothMin(d, s, shape.smoothness); break;
			case ST_BRUSH_SUBTRACT: d = -SmoothMin(-d, s, shape.smoothness); break;
			case ST_BRUSH_INTERSECT: d = -SmoothMin(-d, -s, shape.smoothness); break;
			}
		}
		return d;
	}

	/// Reads one shape of st_sdf_mesh's stream; false with outError on bad input.
	bool ReadSdfShape(const float *&ioCursor, const float *inEnd, uint32_t inIndex, SdfShape &outShape, std::string &outError)
	{
		const std::string name = "shape " + std::to_string(inIndex);
		if (size_t(inEnd - ioCursor) < 19)
		{
			outError = name + ": the stream ends before its kind, operation, smoothness and transform";
			return false;
		}
		outShape.kind = int(ioCursor[0]);
		outShape.operation = int(ioCursor[1]);
		outShape.smoothness = std::max(0.0, double(ioCursor[2]));
		const float *m = ioCursor + 3;
		ioCursor += 19;
		static constexpr int PARAM_COUNTS[] = { 1, 4, 2, 3, 2, 3, 3, 3, 2 };
		if (outShape.kind < 0 || outShape.kind > ST_SDF_HEX_PRISM)
		{
			outError = name + ": unknown kind " + std::to_string(outShape.kind);
			return false;
		}
		if (outShape.operation < ST_BRUSH_ADD || outShape.operation > ST_BRUSH_INTERSECT)
		{
			outError = name + ": unknown operation " + std::to_string(outShape.operation);
			return false;
		}
		const int count = PARAM_COUNTS[outShape.kind];
		if (inEnd - ioCursor < count)
		{
			outError = name + ": the stream ends inside its params";
			return false;
		}
		for (int i = 0; i < count; ++i)
			outShape.params[i] = ioCursor[i];
		ioCursor += count;
		const double *a = outShape.params;
		switch (outShape.kind)
		{
		case ST_SDF_SPHERE: outShape.bound = a[0]; break;
		case ST_SDF_BOX: outShape.bound = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); break;
		case ST_SDF_CAPSULE: outShape.bound = a[0] + a[1]; break;
		case ST_SDF_CYLINDER: outShape.bound = Length2(a[0], a[1]); break;
		case ST_SDF_TORUS: outShape.bound = a[0] + a[1]; break;
		case ST_SDF_ELLIPSOID: outShape.bound = std::max(a[0], std::max(a[1], a[2])); break;
		case ST_SDF_CONE: outShape.bound = Length2(a[0], std::max(a[1], a[2])); break;
		case ST_SDF_ROUND_CONE: outShape.bound = a[0] * 0.5 + std::max(a[1], a[2]); break;
		case ST_SDF_HEX_PRISM: outShape.bound = Length2(a[0], a[1] * 1.1547005383792515); break;
		}
		for (int i = 0; i < count; ++i)
			if (!(a[i] >= 0) || (i == 0 && a[i] <= 0))
			{
				outError = name + ": its sizes must be positive";
				return false;
			}
		if (outShape.kind == ST_SDF_ROUND_CONE && std::abs(a[1] - a[2]) >= a[0])
		{
			outError = name + ": a round cone's radii must differ by less than its height (otherwise one sphere swallows the other)";
			return false;
		}
		if (outShape.kind == ST_SDF_CONE && a[1] <= 0 && a[2] <= 0)
		{
			outError = name + ": a cone needs a bottom or a top radius";
			return false;
		}
		manifold::mat3x4 transform;
		for (int column = 0; column < 4; ++column)
			for (int row = 0; row < 3; ++row)
				transform[column][row] = m[column * 4 + row];
		// A rigid transform with a uniform scale: the scale is the length of a basis column
		const double scale = manifold::la::length(transform[0]);
		if (!(scale > 1e-9))
		{
			outError = name + ": its transform has no scale";
			return false;
		}
		outShape.scale = scale;
		outShape.center = transform[3];
		const manifold::mat3 linear(transform[0], transform[1], transform[2]);
		const manifold::mat3 inverse = manifold::la::inverse(linear);
		outShape.toLocal = manifold::mat3x4(inverse[0], inverse[1], inverse[2], -(inverse * outShape.center));
		return true;
	}
}

SN_API int32_t st_sdf_mesh(const float *shapes, uint32_t float_count, float edge_length)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (shapes == nullptr || float_count == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "shapes are required");
	if (!(edge_length > 0))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "edge_length must be positive");

	return RunTask("SdfMesh", [=](Task &ioTask)
	{
		std::vector<SdfShape> list;
		const float *cursor = shapes, *end = shapes + float_count;
		for (uint32_t index = 0; cursor < end; ++index)
		{
			SdfShape shape;
			std::string error;
			if (!ReadSdfShape(cursor, end, index, shape, error))
			{
				ioTask.Fail(error);
				return false;
			}
			list.push_back(shape);
		}

		// The surface lies inside the added shapes, grown by their blends; subtractions and intersections only take away
		manifold::Box bounds;
		bool any = false;
		for (const SdfShape &shape : list)
		{
			if (shape.operation != ST_BRUSH_ADD)
				continue;
			const double radius = shape.bound * shape.scale + shape.smoothness;
			bounds.Union(shape.center - manifold::vec3(radius));
			bounds.Union(shape.center + manifold::vec3(radius));
			any = true;
		}
		if (!any)
		{
			ioTask.Fail("no shape adds anything (the first shape's operation should be add)");
			return false;
		}
		const double edge = edge_length;
		const manifold::vec3 margin(edge * 2);
		bounds = manifold::Box(bounds.min - margin, bounds.max + margin);
		const manifold::vec3 size = bounds.Size();
		if (std::max(size.x, std::max(size.y, size.z)) / edge > 512)
		{
			ioTask.Fail("edge_length is too small for the shapes' size (more than 512 cells along a side)");
			return false;
		}

		// Manifold's level set is positive inside
		const manifold::Manifold solid = manifold::Manifold::LevelSet([&](manifold::vec3 p) { return -SceneDistance(list, p); }, bounds, edge, 0, -1, false);
		if (solid.Status() != manifold::Manifold::Error::NoError || solid.IsEmpty())
		{
			ioTask.Fail("the shapes make no surface (all subtracted away, or smaller than edge_length)");
			return false;
		}

		const manifold::MeshGL mesh = solid.GetMeshGL();
		const uint32_t vertex_count = uint32_t(mesh.NumVert()), triangle_count = uint32_t(mesh.NumTri());
		float *positions = static_cast<float *>(ioTask.AddOutput(size_t(vertex_count) * 3 * sizeof(float)));
		float *normals = static_cast<float *>(ioTask.AddOutput(size_t(vertex_count) * 3 * sizeof(float)));
		const double h = edge * 0.25;
		for (uint32_t v = 0; v < vertex_count; ++v)
		{
			const manifold::vec3 p(mesh.vertProperties[size_t(v) * mesh.numProp], mesh.vertProperties[size_t(v) * mesh.numProp + 1], mesh.vertProperties[size_t(v) * mesh.numProp + 2]);
			const manifold::vec3 dx(h, 0, 0), dy(0, h, 0), dz(0, 0, h);
			manifold::vec3 gradient(SceneDistance(list, p + dx) - SceneDistance(list, p - dx), SceneDistance(list, p + dy) - SceneDistance(list, p - dy), SceneDistance(list, p + dz) - SceneDistance(list, p - dz));
			const double length = manifold::la::length(gradient);
			gradient = length > 0 ? gradient / length : manifold::vec3(0, 1, 0);
			for (int k = 0; k < 3; ++k)
			{
				positions[v * 3 + k] = float(p[k]);
				normals[v * 3 + k] = float(gradient[k]);
			}
		}
		uint32_t *indices = static_cast<uint32_t *>(ioTask.AddOutput(size_t(triangle_count) * 3 * sizeof(uint32_t)));
		std::memcpy(indices, mesh.triVerts.data(), size_t(triangle_count) * 3 * sizeof(uint32_t));
		uint32_t *owner = static_cast<uint32_t *>(ioTask.AddOutput(size_t(triangle_count) * sizeof(uint32_t)));
		for (uint32_t t = 0; t < triangle_count; ++t)
		{
			manifold::vec3 centroid(0.0);
			for (int c = 0; c < 3; ++c)
				centroid += manifold::vec3(positions[indices[t * 3 + c] * 3], positions[indices[t * 3 + c] * 3 + 1], positions[indices[t * 3 + c] * 3 + 2]);
			centroid /= 3.0;
			double nearest = 1e18;
			for (uint32_t s = 0; s < list.size(); ++s)
			{
				const double d = std::abs(ShapeDistance(list[s], centroid));
				if (d < nearest)
				{
					nearest = d;
					owner[t] = s;
				}
			}
		}
		ioTask.SetResult(0, int32_t(triangle_count));
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
