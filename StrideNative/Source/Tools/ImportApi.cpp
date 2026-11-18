// 场景导入：glTF/GLB 用 cgltf，FBX 用 ufbx，都化成带索引的三角形网格。
// 和 Stride 的模型导入一致：每个节点实例上的网格按材质拆开（一份材质一个网格，记下材质在场景材质表里的下标），
// 节点的世界变换烘焙进顶点和法线（镜像变换时翻转绕序），所以网格都在模型空间里，右手、Y 向上、米。
#include <Core/Core.h>
#include <Core/HandleTable.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <cgltf.h>
#include <meshoptimizer.h>
#include <ufbx.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <string>
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
		uint32_t				mMaterial = ST_NO_MATERIAL;
	};

	struct ImportedScene
	{
		std::vector<ImportedMesh>	mMeshes;
		std::vector<std::string>	mMaterials;		///< 材质名，按网格的 mMaterial 下标
	};

	/// 不带索引的角顶点流，用 meshoptimizer 焊接成带索引的网格。
	struct Vertex
	{
		float mPosition[3];
		float mNormal[3];
		float mUv[2];
	};

	ImportedMesh WeldCorners(const std::vector<Vertex> &inCorners, bool inHasNormals, bool inHasUvs, uint32_t inMaterial)
	{
		std::vector<uint32_t> remap(inCorners.size());
		size_t vertex_count = meshopt_generateVertexRemap(remap.data(), nullptr, inCorners.size(), inCorners.data(), inCorners.size(), sizeof(Vertex));
		std::vector<Vertex> vertices(vertex_count);
		meshopt_remapVertexBuffer(vertices.data(), inCorners.data(), inCorners.size(), sizeof(Vertex), remap.data());

		ImportedMesh mesh;
		mesh.mIndices = std::move(remap);
		mesh.mMaterial = inMaterial;
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

	/// 列主序 4x4（glTF 的布局）作用到点和法线上；法线用左上 3x3 的逆转置。
	struct NodeTransform
	{
		float	mMatrix[16];
		float	mNormal[9];
		bool	mMirrored;

		explicit NodeTransform(const float *inMatrix)
		{
			std::memcpy(mMatrix, inMatrix, sizeof(mMatrix));
			const float *m = mMatrix;
			// 3x3 的余子式矩阵就是 det * 逆的转置
			float c00 = m[5] * m[10] - m[9] * m[6], c01 = m[9] * m[2] - m[1] * m[10], c02 = m[1] * m[6] - m[5] * m[2];
			float c10 = m[8] * m[6] - m[4] * m[10], c11 = m[0] * m[10] - m[8] * m[2], c12 = m[4] * m[2] - m[0] * m[6];
			float c20 = m[4] * m[9] - m[8] * m[5], c21 = m[8] * m[1] - m[0] * m[9], c22 = m[0] * m[5] - m[4] * m[1];
			float det = m[0] * c00 + m[4] * c01 + m[8] * c02;
			mMirrored = det < 0;
			// 法线矩阵 = 逆的转置，按列主序存：第 j 列第 i 行是 cof(i, j) / det
			float s = det != 0 ? 1.0f / det : 0.0f;
			const float normal[9] = { c00 * s, c01 * s, c02 * s, c10 * s, c11 * s, c12 * s, c20 * s, c21 * s, c22 * s };
			std::memcpy(mNormal, normal, sizeof(mNormal));
		}

		void Point(const float *inP, float *outP) const
		{
			for (int r = 0; r < 3; ++r)
				outP[r] = mMatrix[r] * inP[0] + mMatrix[4 + r] * inP[1] + mMatrix[8 + r] * inP[2] + mMatrix[12 + r];
		}

		void Normal(const float *inN, float *outN) const
		{
			for (int r = 0; r < 3; ++r)
				outN[r] = mNormal[r * 3] * inN[0] + mNormal[r * 3 + 1] * inN[1] + mNormal[r * 3 + 2] * inN[2];
			float length = std::sqrt(outN[0] * outN[0] + outN[1] * outN[1] + outN[2] * outN[2]);
			if (length > 0)
				for (int r = 0; r < 3; ++r)
					outN[r] /= length;
		}
	};

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

		for (cgltf_size i = 0; i < data->materials_count; ++i)
			outScene.mMaterials.emplace_back(data->materials[i].name != nullptr ? data->materials[i].name : "material" + std::to_string(i));

		auto import_mesh = [&](const cgltf_mesh &inMesh, const NodeTransform &inTransform)
		{
			for (cgltf_size p = 0; p < inMesh.primitives_count; ++p)
			{
				const cgltf_primitive &primitive = inMesh.primitives[p];
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
				mesh.mMaterial = primitive.material != nullptr ? uint32_t(primitive.material - data->materials) : ST_NO_MATERIAL;
				std::vector<float> local(positions->count * 3);
				cgltf_accessor_unpack_floats(positions, local.data(), local.size());
				mesh.mPositions.resize(local.size());
				for (size_t v = 0; v < positions->count; ++v)
					inTransform.Point(&local[v * 3], &mesh.mPositions[v * 3]);
				if (normals != nullptr)
				{
					local.resize(normals->count * 3);
					cgltf_accessor_unpack_floats(normals, local.data(), local.size());
					mesh.mNormals.resize(local.size());
					for (size_t v = 0; v < normals->count; ++v)
						inTransform.Normal(&local[v * 3], &mesh.mNormals[v * 3]);
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
				if (inTransform.mMirrored)
					for (size_t i = 0; i + 2 < mesh.mIndices.size(); i += 3)
						std::swap(mesh.mIndices[i + 1], mesh.mIndices[i + 2]);
				outScene.mMeshes.push_back(std::move(mesh));
			}
		};

		// 每个挂着网格的节点是一个实例；没有节点引用网格的文件（少见）按单位变换导入每个网格
		bool any_node = false;
		for (cgltf_size n = 0; n < data->nodes_count; ++n)
		{
			const cgltf_node &node = data->nodes[n];
			if (node.mesh == nullptr)
				continue;
			any_node = true;
			float world[16];
			cgltf_node_transform_world(&node, world);
			import_mesh(*node.mesh, NodeTransform(world));
		}
		if (!any_node)
		{
			const float identity[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
			for (cgltf_size m = 0; m < data->meshes_count; ++m)
				import_mesh(data->meshes[m], NodeTransform(identity));
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

		for (size_t i = 0; i < scene->materials.count; ++i)
			outScene.mMaterials.emplace_back(scene->materials.data[i]->name.data);

		for (size_t n = 0; n < scene->nodes.count; ++n)
		{
			const ufbx_node &node = *scene->nodes.data[n];
			if (node.mesh == nullptr)
				continue;
			const ufbx_mesh &mesh = *node.mesh;
			const ufbx_matrix &to_world = node.geometry_to_world;
			ufbx_matrix to_world_normals = ufbx_matrix_for_normals(&to_world);
			bool mirrored = ufbx_matrix_determinant(&to_world) < 0;
			std::vector<uint32_t> triangle(mesh.max_face_triangles * 3);

			// 一个材质分段一个网格；实例上的材质（node.materials）优先，没有时用网格自己的
			for (size_t p = 0; p < mesh.material_parts.count; ++p)
			{
				const ufbx_mesh_part &part = mesh.material_parts.data[p];
				if (part.num_triangles == 0)
					continue;
				const ufbx_material *material = part.index < node.materials.count ? node.materials.data[part.index]
					: part.index < mesh.materials.count ? mesh.materials.data[part.index] : nullptr;

				std::vector<Vertex> corners;
				corners.reserve(part.num_triangles * 3);
				for (size_t f = 0; f < part.face_indices.count; ++f)
				{
					uint32_t triangle_count = ufbx_triangulate_face(triangle.data(), triangle.size(), &mesh, mesh.faces.data[part.face_indices.data[f]]);
					for (uint32_t c = 0; c < triangle_count * 3; ++c)
					{
						// 镜像变换翻转绕序：每个三角形的后两个角对调
						uint32_t corner = mirrored && c % 3 != 0 ? (c % 3 == 1 ? c + 1 : c - 1) : c;
						uint32_t index = triangle[corner];
						Vertex vertex {};
						ufbx_vec3 position = ufbx_transform_position(&to_world, ufbx_get_vertex_vec3(&mesh.vertex_position, index));
						vertex.mPosition[0] = float(position.x); vertex.mPosition[1] = float(position.y); vertex.mPosition[2] = float(position.z);
						if (mesh.vertex_normal.exists)
						{
							ufbx_vec3 normal = ufbx_transform_direction(&to_world_normals, ufbx_get_vertex_vec3(&mesh.vertex_normal, index));
							double length = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
							if (length > 0)
							{
								normal.x /= length;
								normal.y /= length;
								normal.z /= length;
							}
							vertex.mNormal[0] = float(normal.x); vertex.mNormal[1] = float(normal.y); vertex.mNormal[2] = float(normal.z);
						}
						if (mesh.vertex_uv.exists)
						{
							ufbx_vec2 uv = ufbx_get_vertex_vec2(&mesh.vertex_uv, index);
							vertex.mUv[0] = float(uv.x);
							vertex.mUv[1] = 1.0f - float(uv.y); // FBX 的 UV 从下边起，glTF/WebGPU 从上边起
						}
						corners.push_back(vertex);
					}
				}
				uint32_t material_index = material != nullptr ? material->typed_id : ST_NO_MATERIAL;
				outScene.mMeshes.push_back(WeldCorners(corners, mesh.vertex_normal.exists, mesh.vertex_uv.exists, material_index));
			}
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
		ioTask.SetResult(2, int32_t(scene->mMaterials.size()));
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
	out_info->material = m.mMaterial;
	return SN_OK;
}

SN_API const char *st_scene_material_name(int32_t scene, uint32_t material)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return nullptr;
	if (material >= s->mMaterials.size())
	{
		SetLastError(SN_ERROR_INVALID_ARGUMENT, "material index out of range");
		return nullptr;
	}
	return s->mMaterials[material].c_str();
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
