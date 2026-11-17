// Scene import: glTF/GLB through cgltf, FBX through ufbx, both reduced to indexed triangle meshes.
#include <Core/Core.h>
#include <Core/HandleTable.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <cgltf.h>
#include <meshoptimizer.h>
#include <ufbx.h>

#include <cstring>
#include <memory>
#include <vector>

namespace
{
	using namespace StrideNative;

	struct ImportedMesh
	{
		std::vector<float>		mPositions;
		std::vector<float>		mNormals;
		std::vector<float>		mUvs;
		std::vector<uint32_t>	mIndices;
	};

	struct ImportedScene
	{
		std::vector<ImportedMesh> mMeshes;
	};

	/// Corner (unindexed) vertex stream, welded into an indexed mesh with meshoptimizer.
	struct Vertex
	{
		float mPosition[3];
		float mNormal[3];
		float mUv[2];
	};

	ImportedMesh WeldCorners(const std::vector<Vertex> &inCorners, bool inHasNormals, bool inHasUvs)
	{
		std::vector<uint32_t> remap(inCorners.size());
		size_t vertex_count = meshopt_generateVertexRemap(remap.data(), nullptr, inCorners.size(), inCorners.data(), inCorners.size(), sizeof(Vertex));
		std::vector<Vertex> vertices(vertex_count);
		meshopt_remapVertexBuffer(vertices.data(), inCorners.data(), inCorners.size(), sizeof(Vertex), remap.data());

		ImportedMesh mesh;
		mesh.mIndices = std::move(remap);
		for (const Vertex &v : vertices)
		{
			mesh.mPositions.insert(mesh.mPositions.end(), v.mPosition, v.mPosition + 3);
			if (inHasNormals)
				mesh.mNormals.insert(mesh.mNormals.end(), v.mNormal, v.mNormal + 3);
			if (inHasUvs)
				mesh.mUvs.insert(mesh.mUvs.end(), v.mUv, v.mUv + 2);
		}
		return mesh;
	}

	bool ImportGltf(const void *inData, uint32_t inSize, ImportedScene &outScene, std::string &outError)
	{
		cgltf_options options {};
		cgltf_data *data = nullptr;
		if (cgltf_parse(&options, inData, inSize, &data) != cgltf_result_success)
		{
			outError = "not a valid glTF/GLB file";
			return false;
		}
		std::unique_ptr<cgltf_data, decltype(&cgltf_free)> owned(data, &cgltf_free);
		if (cgltf_load_buffers(&options, data, nullptr) != cgltf_result_success)
		{
			outError = "glTF buffers must be embedded (GLB or data URIs)";
			return false;
		}

		for (cgltf_size m = 0; m < data->meshes_count; ++m)
			for (cgltf_size p = 0; p < data->meshes[m].primitives_count; ++p)
			{
				const cgltf_primitive &primitive = data->meshes[m].primitives[p];
				if (primitive.type != cgltf_primitive_type_triangles)
					continue;

				const cgltf_accessor *positions = nullptr, *normals = nullptr, *uvs = nullptr;
				for (cgltf_size a = 0; a < primitive.attributes_count; ++a)
				{
					const cgltf_attribute &attribute = primitive.attributes[a];
					if (attribute.type == cgltf_attribute_type_position)
						positions = attribute.data;
					else if (attribute.type == cgltf_attribute_type_normal)
						normals = attribute.data;
					else if (attribute.type == cgltf_attribute_type_texcoord && attribute.index == 0)
						uvs = attribute.data;
				}
				if (positions == nullptr)
					continue;

				ImportedMesh mesh;
				mesh.mPositions.resize(positions->count * 3);
				cgltf_accessor_unpack_floats(positions, mesh.mPositions.data(), mesh.mPositions.size());
				if (normals != nullptr)
				{
					mesh.mNormals.resize(normals->count * 3);
					cgltf_accessor_unpack_floats(normals, mesh.mNormals.data(), mesh.mNormals.size());
				}
				if (uvs != nullptr)
				{
					mesh.mUvs.resize(uvs->count * 2);
					cgltf_accessor_unpack_floats(uvs, mesh.mUvs.data(), mesh.mUvs.size());
				}
				if (primitive.indices != nullptr)
				{
					mesh.mIndices.resize(primitive.indices->count);
					cgltf_accessor_unpack_indices(primitive.indices, mesh.mIndices.data(), sizeof(uint32_t), mesh.mIndices.size());
				}
				else
				{
					mesh.mIndices.resize(positions->count);
					for (uint32_t i = 0; i < mesh.mIndices.size(); ++i)
						mesh.mIndices[i] = i;
				}
				outScene.mMeshes.push_back(std::move(mesh));
			}
		return true;
	}

	bool ImportFbx(const void *inData, uint32_t inSize, ImportedScene &outScene, std::string &outError)
	{
		ufbx_load_opts options {};
		options.target_axes = ufbx_axes_right_handed_y_up;
		options.target_unit_meters = 1.0f;
		options.space_conversion = UFBX_SPACE_CONVERSION_MODIFY_GEOMETRY;
		options.generate_missing_normals = true;

		ufbx_error error;
		ufbx_scene *scene = ufbx_load_memory(inData, inSize, &options, &error);
		if (scene == nullptr)
		{
			outError = std::string("FBX load failed: ") + error.description.data;
			return false;
		}
		std::unique_ptr<ufbx_scene, decltype(&ufbx_free_scene)> owned(scene, &ufbx_free_scene);

		for (size_t m = 0; m < scene->meshes.count; ++m)
		{
			const ufbx_mesh &mesh = *scene->meshes.data[m];
			std::vector<uint32_t> triangle(mesh.max_face_triangles * 3);
			std::vector<Vertex> corners;
			corners.reserve(mesh.num_triangles * 3);
			for (size_t f = 0; f < mesh.faces.count; ++f)
			{
				uint32_t triangle_count = ufbx_triangulate_face(triangle.data(), triangle.size(), &mesh, mesh.faces.data[f]);
				for (uint32_t c = 0; c < triangle_count * 3; ++c)
				{
					uint32_t index = triangle[c];
					Vertex vertex {};
					ufbx_vec3 position = ufbx_get_vertex_vec3(&mesh.vertex_position, index);
					vertex.mPosition[0] = float(position.x); vertex.mPosition[1] = float(position.y); vertex.mPosition[2] = float(position.z);
					if (mesh.vertex_normal.exists)
					{
						ufbx_vec3 normal = ufbx_get_vertex_vec3(&mesh.vertex_normal, index);
						vertex.mNormal[0] = float(normal.x); vertex.mNormal[1] = float(normal.y); vertex.mNormal[2] = float(normal.z);
					}
					if (mesh.vertex_uv.exists)
					{
						ufbx_vec2 uv = ufbx_get_vertex_vec2(&mesh.vertex_uv, index);
						vertex.mUv[0] = float(uv.x);
						vertex.mUv[1] = 1.0f - float(uv.y); // FBX UVs start at the bottom, glTF/WebGPU at the top
					}
					corners.push_back(vertex);
				}
			}
			if (!corners.empty())
				outScene.mMeshes.push_back(WeldCorners(corners, mesh.vertex_normal.exists, mesh.vertex_uv.exists));
		}
		return true;
	}

	bool LooksLikeFbx(const void *inData, uint32_t inSize)
	{
		static const char cBinaryMagic[] = "Kaydara FBX Binary";
		if (inSize >= sizeof(cBinaryMagic) - 1 && std::memcmp(inData, cBinaryMagic, sizeof(cBinaryMagic) - 1) == 0)
			return true;
		// ASCII FBX files start with a "; FBX" comment line.
		return inSize >= 5 && std::memcmp(inData, "; FBX", 5) == 0;
	}
}

SN_API int32_t st_scene_load(const void *data, uint32_t size, int32_t format)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (data == nullptr || size == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "scene data is empty");
	if (format == ST_FORMAT_AUTO)
		format = LooksLikeFbx(data, size) ? ST_FORMAT_FBX : ST_FORMAT_GLTF;
	if (format != ST_FORMAT_GLTF && format != ST_FORMAT_FBX)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "unknown scene format " + std::to_string(format));

	return RunTask("SceneLoad", [data, size, format](Task &ioTask)
	{
		auto scene = std::make_unique<ImportedScene>();
		std::string error;
		bool loaded = format == ST_FORMAT_FBX ? ImportFbx(data, size, *scene, error) : ImportGltf(data, size, *scene, error);
		if (!loaded)
		{
			ioTask.Fail(error);
			return false;
		}
		ioTask.SetResult(1, int32_t(scene->mMeshes.size()));
		ioTask.SetResult(0, GetTable<ImportedScene>().Add(std::move(scene)));
		return true;
	});
}

SN_API void st_scene_destroy(int32_t scene)
{
	GetTable<ImportedScene>().Remove(scene);
}

SN_API int32_t st_scene_mesh_info(int32_t scene, uint32_t mesh, st_mesh_info *out_info)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (mesh >= s->mMeshes.size() || out_info == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "mesh index out of range or out_info is null");

	const ImportedMesh &m = s->mMeshes[mesh];
	out_info->vertex_count = uint32_t(m.mPositions.size() / 3);
	out_info->index_count = uint32_t(m.mIndices.size());
	out_info->has_normals = !m.mNormals.empty();
	out_info->has_uvs = !m.mUvs.empty();
	return SN_OK;
}

SN_API int32_t st_scene_mesh_copy(int32_t scene, uint32_t mesh, float *positions, float *normals, float *uvs, uint32_t *indices)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (mesh >= s->mMeshes.size())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "mesh index out of range");

	const ImportedMesh &m = s->mMeshes[mesh];
	auto copy = [](auto *outTarget, const auto &inSource)
	{
		if (outTarget != nullptr && !inSource.empty())
			std::memcpy(outTarget, inSource.data(), inSource.size() * sizeof(inSource[0]));
	};
	copy(positions, m.mPositions);
	copy(normals, m.mNormals);
	copy(uvs, m.mUvs);
	copy(indices, m.mIndices);
	return SN_OK;
}
