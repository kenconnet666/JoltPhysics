// 场景导入：glTF/GLB 用 cgltf，FBX 用 ufbx，都化成带索引的三角形网格，加上节点层级、蒙皮和（FBX 的）动画。
// 和 Stride 的模型导入一致：每个节点实例上的网格按材质拆开（一份材质一个网格，记下材质在场景材质表里的下标），
// 节点的世界变换烘焙进顶点和法线（镜像变换时翻转绕序），所以网格都在模型空间里，右手、Y 向上、米。
// 蒙皮网格按绑定时的姿势烘焙；每个关节的逆绑定矩阵把烘焙后的顶点变到关节空间，关节当前的节点到模型矩阵乘它就是蒙皮矩阵。
// 节点按深度优先排（父节点在子节点前面，同 Stride 的 .sdskel 和 ozz 的骨架），FBX 的根节点叫 RootNode（同 Stride 的导入）。
#include <Core/Core.h>
#include <Core/HandleTable.h>
#include <Core/Tasks.h>
#include <StrideNative/Tools.h>

#include <cgltf.h>
#include <meshoptimizer.h>
#include <ufbx.h>

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

	constexpr int cMaxInfluences = 4;

	struct ImportedNode
	{
		std::string				mName;
		int32_t					mParent = -1;
		float					mTranslation[3] = { 0, 0, 0 };
		float					mRotation[4] = { 0, 0, 0, 1 };
		float					mScale[3] = { 1, 1, 1 };
		float					mWorld[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
	};

	struct ImportedMesh
	{
		std::vector<float>		mPositions;
		std::vector<float>		mNormals;
		std::vector<float>		mUvs;
		std::vector<uint32_t>	mIndices;
		uint32_t				mMaterial = ST_NO_MATERIAL;
		uint32_t				mNode = 0;
		// 蒙皮：影响这个网格的关节（节点下标）、每个关节的逆绑定矩阵（列主序），每个顶点 4 个关节下标和权重
		std::vector<uint32_t>	mJointNodes;
		std::vector<float>		mInverseBinds;
		std::vector<uint16_t>	mJoints;
		std::vector<float>		mWeights;
	};

	struct ImportedScene
	{
		std::vector<ImportedMesh>	mMeshes;
		std::vector<std::string>	mMaterials;		///< 材质名，按网格的 mMaterial 下标
		std::vector<ImportedNode>	mNodes;
		std::vector<std::string>	mAnimations;
		/// FBX 的场景留着给 st_scene_animation_bake 用；mFbxNodes[i] 是我们的第 i 个节点
		std::shared_ptr<ufbx_scene>	mFbx;
		std::vector<const ufbx_node *> mFbxNodes;
	};

	/// 不带索引的角顶点流，用 meshoptimizer 焊接成带索引的网格。蒙皮数据也参与比较：同一个位置的角权重不同就不合并。
	struct Vertex
	{
		float		mPosition[3];
		float		mNormal[3];
		float		mUv[2];
		uint16_t	mJoints[cMaxInfluences];
		float		mWeights[cMaxInfluences];
	};

	ImportedMesh WeldCorners(const std::vector<Vertex> &inCorners, bool inHasNormals, bool inHasUvs, bool inSkinned, uint32_t inMaterial)
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
			if (inSkinned)
			{
				mesh.mJoints.insert(mesh.mJoints.end(), v.mJoints, v.mJoints + cMaxInfluences);
				mesh.mWeights.insert(mesh.mWeights.end(), v.mWeights, v.mWeights + cMaxInfluences);
			}
		}
		return mesh;
	}

	/// 一个顶点的影响：按权重从大到小留前 4 个，归一化。
	void StoreInfluences(std::vector<std::pair<float, uint16_t>> &ioInfluences, Vertex &ioVertex)
	{
		std::sort(ioInfluences.begin(), ioInfluences.end(), [](const auto &inA, const auto &inB) { return inA.first > inB.first; });
		float total = 0;
		for (size_t i = 0; i < ioInfluences.size() && i < cMaxInfluences; ++i)
			total += ioInfluences[i].first;
		for (int i = 0; i < cMaxInfluences; ++i)
		{
			bool used = size_t(i) < ioInfluences.size() && total > 0;
			ioVertex.mJoints[i] = used ? ioInfluences[i].second : 0;
			ioVertex.mWeights[i] = used ? ioInfluences[i].first / total : 0.0f;
		}
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

	/// 列主序 4x4 拆成平移、旋转（xyzw）、缩放；没有切变的矩阵（glTF 节点的 matrix）才精确。
	void DecomposeMatrix(const float *inM, float *outT, float *outR, float *outS)
	{
		outT[0] = inM[12]; outT[1] = inM[13]; outT[2] = inM[14];
		float axes[3][3];
		for (int c = 0; c < 3; ++c)
		{
			outS[c] = std::sqrt(inM[c * 4] * inM[c * 4] + inM[c * 4 + 1] * inM[c * 4 + 1] + inM[c * 4 + 2] * inM[c * 4 + 2]);
			for (int r = 0; r < 3; ++r)
				axes[c][r] = outS[c] != 0 ? inM[c * 4 + r] / outS[c] : 0.0f;
		}
		// 旋转矩阵转四元数（Shepperd 的方法，按最大的对角项分情况）
		float m00 = axes[0][0], m11 = axes[1][1], m22 = axes[2][2], trace = m00 + m11 + m22;
		float x, y, z, w;
		if (trace > 0)
		{
			float s = std::sqrt(trace + 1.0f) * 2;
			w = 0.25f * s; x = (axes[1][2] - axes[2][1]) / s; y = (axes[2][0] - axes[0][2]) / s; z = (axes[0][1] - axes[1][0]) / s;
		}
		else if (m00 > m11 && m00 > m22)
		{
			float s = std::sqrt(1.0f + m00 - m11 - m22) * 2;
			w = (axes[1][2] - axes[2][1]) / s; x = 0.25f * s; y = (axes[1][0] + axes[0][1]) / s; z = (axes[2][0] + axes[0][2]) / s;
		}
		else if (m11 > m22)
		{
			float s = std::sqrt(1.0f + m11 - m00 - m22) * 2;
			w = (axes[2][0] - axes[0][2]) / s; x = (axes[1][0] + axes[0][1]) / s; y = 0.25f * s; z = (axes[2][1] + axes[1][2]) / s;
		}
		else
		{
			float s = std::sqrt(1.0f + m22 - m00 - m11) * 2;
			w = (axes[0][1] - axes[1][0]) / s; x = (axes[2][0] + axes[0][2]) / s; y = (axes[2][1] + axes[1][2]) / s; z = 0.25f * s;
		}
		outR[0] = x; outR[1] = y; outR[2] = z; outR[3] = w;
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

		for (cgltf_size i = 0; i < data->materials_count; ++i)
			outScene.mMaterials.emplace_back(data->materials[i].name != nullptr ? data->materials[i].name : "material" + std::to_string(i));

		// 节点按深度优先排：从没有父节点的节点开始
		std::vector<uint32_t> index_of(data->nodes_count, 0);
		auto add_node = [&](auto &&inSelf, const cgltf_node &inNode, int32_t inParent) -> void
		{
			uint32_t index = uint32_t(outScene.mNodes.size());
			index_of[size_t(&inNode - data->nodes)] = index;
			ImportedNode &node = outScene.mNodes.emplace_back();
			node.mName = inNode.name != nullptr ? inNode.name : "node" + std::to_string(&inNode - data->nodes);
			node.mParent = inParent;
			float local[16];
			cgltf_node_transform_local(&inNode, local);
			DecomposeMatrix(local, node.mTranslation, node.mRotation, node.mScale);
			cgltf_node_transform_world(&inNode, node.mWorld);
			for (cgltf_size c = 0; c < inNode.children_count; ++c)
				inSelf(inSelf, *inNode.children[c], int32_t(index));
		};
		for (cgltf_size n = 0; n < data->nodes_count; ++n)
			if (data->nodes[n].parent == nullptr)
				add_node(add_node, data->nodes[n], -1);

		auto import_mesh = [&](const cgltf_mesh &inMesh, const NodeTransform &inTransform, uint32_t inNode, const cgltf_skin *inSkin)
		{
			for (cgltf_size p = 0; p < inMesh.primitives_count; ++p)
			{
				const cgltf_primitive &primitive = inMesh.primitives[p];
				if (primitive.type != cgltf_primitive_type_triangles)
					continue;

				const cgltf_accessor *positions = nullptr, *normals = nullptr, *uvs = nullptr, *joints = nullptr, *weights = nullptr;
				for (cgltf_size a = 0; a < primitive.attributes_count; ++a)
				{
					const cgltf_attribute &attribute = primitive.attributes[a];
					if (attribute.type == cgltf_attribute_type_position)
						positions = attribute.data;
					else if (attribute.type == cgltf_attribute_type_normal)
						normals = attribute.data;
					else if (attribute.type == cgltf_attribute_type_texcoord && attribute.index == 0)
						uvs = attribute.data;
					else if (attribute.type == cgltf_attribute_type_joints && attribute.index == 0)
						joints = attribute.data;
					else if (attribute.type == cgltf_attribute_type_weights && attribute.index == 0)
						weights = attribute.data;
				}
				if (positions == nullptr)
					continue;

				ImportedMesh mesh;
				mesh.mMaterial = primitive.material != nullptr ? uint32_t(primitive.material - data->materials) : ST_NO_MATERIAL;
				mesh.mNode = inNode;
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
				if (inSkin != nullptr && joints != nullptr && weights != nullptr)
				{
					// glTF 的蒙皮网格不受自己节点的变换影响（规范如此），所以按单位变换烘焙；逆绑定矩阵直接用
					for (cgltf_size j = 0; j < inSkin->joints_count; ++j)
					{
						mesh.mJointNodes.push_back(index_of[size_t(inSkin->joints[j] - data->nodes)]);
						float inverse_bind[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
						if (inSkin->inverse_bind_matrices != nullptr)
							cgltf_accessor_read_float(inSkin->inverse_bind_matrices, j, inverse_bind, 16);
						mesh.mInverseBinds.insert(mesh.mInverseBinds.end(), inverse_bind, inverse_bind + 16);
					}
					mesh.mJoints.resize(positions->count * cMaxInfluences);
					mesh.mWeights.resize(positions->count * cMaxInfluences);
					for (size_t v = 0; v < positions->count; ++v)
					{
						cgltf_uint joint[4] = {};
						float weight[4] = {};
						cgltf_accessor_read_uint(joints, v, joint, 4);
						cgltf_accessor_read_float(weights, v, weight, 4);
						std::vector<std::pair<float, uint16_t>> influences;
						for (int k = 0; k < 4; ++k)
							if (weight[k] > 0)
								influences.emplace_back(weight[k], uint16_t(joint[k]));
						Vertex vertex {};
						StoreInfluences(influences, vertex);
						std::memcpy(&mesh.mJoints[v * cMaxInfluences], vertex.mJoints, sizeof(vertex.mJoints));
						std::memcpy(&mesh.mWeights[v * cMaxInfluences], vertex.mWeights, sizeof(vertex.mWeights));
					}
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
		const float identity[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
		bool any_node = false;
		for (cgltf_size n = 0; n < data->nodes_count; ++n)
		{
			const cgltf_node &node = data->nodes[n];
			if (node.mesh == nullptr)
				continue;
			any_node = true;
			float world[16];
			cgltf_node_transform_world(&node, world);
			import_mesh(*node.mesh, NodeTransform(node.skin != nullptr ? identity : world), index_of[n], node.skin);
		}
		if (!any_node)
			for (cgltf_size m = 0; m < data->meshes_count; ++m)
				import_mesh(data->meshes[m], NodeTransform(identity), 0, nullptr);
		return true;
	}

	void StoreMatrix(const ufbx_matrix &inMatrix, float *outMatrix)
	{
		for (int c = 0; c < 4; ++c)
		{
			outMatrix[c * 4 + 0] = float(inMatrix.cols[c].x);
			outMatrix[c * 4 + 1] = float(inMatrix.cols[c].y);
			outMatrix[c * 4 + 2] = float(inMatrix.cols[c].z);
			outMatrix[c * 4 + 3] = c == 3 ? 1.0f : 0.0f;
		}
	}

	/// FBX 和 OBJ 都由 ufbx 读；OBJ 不带坐标轴和单位，按右手 Y 向上、米（不做转换）
	bool ImportUfbx(const void *inData, uint32_t inSize, bool inObj, ImportedScene &outScene, std::string &outError)
	{
		ufbx_load_opts options {};
		options.target_axes = ufbx_axes_right_handed_y_up;
		options.target_unit_meters = 1.0f;
		options.space_conversion = UFBX_SPACE_CONVERSION_MODIFY_GEOMETRY;
		options.generate_missing_normals = true;
		if (inObj)
		{
			options.file_format = UFBX_FILE_FORMAT_OBJ;
			options.obj_axes = ufbx_axes_right_handed_y_up;
			options.obj_unit_meters = 1.0f;
		}

		ufbx_error error;
		ufbx_scene *scene = ufbx_load_memory(inData, inSize, &options, &error);
		if (scene == nullptr)
		{
			outError = std::string(inObj ? "OBJ" : "FBX") + " load failed: " + error.description.data;
			return false;
		}
		outScene.mFbx = std::shared_ptr<ufbx_scene>(scene, &ufbx_free_scene);

		for (size_t i = 0; i < scene->materials.count; ++i)
			outScene.mMaterials.emplace_back(scene->materials.data[i]->name.data);
		for (size_t i = 0; i < scene->anim_stacks.count; ++i)
			outScene.mAnimations.emplace_back(scene->anim_stacks.data[i]->name.data);

		// 节点按深度优先排；ufbx 的 typed_id 换成我们的下标
		std::unordered_map<uint32_t, uint32_t> index_of;
		auto add_node = [&](auto &&inSelf, const ufbx_node &inNode, int32_t inParent) -> void
		{
			uint32_t index = uint32_t(outScene.mNodes.size());
			index_of[inNode.typed_id] = index;
			outScene.mFbxNodes.push_back(&inNode);
			ImportedNode &node = outScene.mNodes.emplace_back();
			node.mName = inNode.is_root ? "RootNode" : inNode.name.data;
			node.mParent = inParent;
			const ufbx_transform &local = inNode.local_transform;
			node.mTranslation[0] = float(local.translation.x); node.mTranslation[1] = float(local.translation.y); node.mTranslation[2] = float(local.translation.z);
			node.mRotation[0] = float(local.rotation.x); node.mRotation[1] = float(local.rotation.y); node.mRotation[2] = float(local.rotation.z); node.mRotation[3] = float(local.rotation.w);
			node.mScale[0] = float(local.scale.x); node.mScale[1] = float(local.scale.y); node.mScale[2] = float(local.scale.z);
			StoreMatrix(inNode.node_to_world, node.mWorld);
			for (size_t c = 0; c < inNode.children.count; ++c)
				inSelf(inSelf, *inNode.children.data[c], int32_t(index));
		};
		add_node(add_node, *scene->root_node, -1);

		for (size_t n = 0; n < scene->nodes.count; ++n)
		{
			const ufbx_node &node = *scene->nodes.data[n];
			if (node.mesh == nullptr)
				continue;
			const ufbx_mesh &mesh = *node.mesh;
			// 蒙皮网格按绑定时的变换烘焙（第一个簇记着绑定时几何到世界的矩阵），其余按节点现在的变换
			const ufbx_skin_deformer *skin = mesh.skin_deformers.count > 0 && mesh.skin_deformers.data[0]->clusters.count > 0 ? mesh.skin_deformers.data[0] : nullptr;
			const ufbx_matrix to_world = skin != nullptr ? skin->clusters.data[0]->geometry_to_world : node.geometry_to_world;
			ufbx_matrix to_world_normals = ufbx_matrix_for_normals(&to_world);
			bool mirrored = ufbx_matrix_determinant(&to_world) < 0;
			std::vector<uint32_t> triangle(mesh.max_face_triangles * 3);

			std::vector<uint32_t> joint_nodes;
			std::vector<float> inverse_binds;
			if (skin != nullptr)
			{
				// 逆绑定矩阵：烘焙后的顶点 → 几何空间（to_world 的逆）→ 骨骼空间（geometry_to_bone）
				ufbx_matrix from_world = ufbx_matrix_invert(&to_world);
				for (size_t c = 0; c < skin->clusters.count; ++c)
				{
					const ufbx_skin_cluster &cluster = *skin->clusters.data[c];
					joint_nodes.push_back(cluster.bone_node != nullptr ? index_of[cluster.bone_node->typed_id] : 0);
					ufbx_matrix inverse_bind = ufbx_matrix_mul(&cluster.geometry_to_bone, &from_world);
					float stored[16];
					StoreMatrix(inverse_bind, stored);
					inverse_binds.insert(inverse_binds.end(), stored, stored + 16);
				}
			}

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
				std::vector<std::pair<float, uint16_t>> influences;
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
						if (skin != nullptr)
						{
							const ufbx_skin_vertex &weights = skin->vertices.data[mesh.vertex_indices.data[index]];
							influences.clear();
							for (uint32_t w = 0; w < weights.num_weights; ++w)
							{
								const ufbx_skin_weight &weight = skin->weights.data[weights.weight_begin + w];
								influences.emplace_back(float(weight.weight), uint16_t(weight.cluster_index));
							}
							StoreInfluences(influences, vertex);
						}
						corners.push_back(vertex);
					}
				}
				uint32_t material_index = material != nullptr ? material->typed_id : ST_NO_MATERIAL;
				ImportedMesh imported = WeldCorners(corners, mesh.vertex_normal.exists, mesh.vertex_uv.exists, skin != nullptr, material_index);
				imported.mNode = index_of[node.typed_id];
				imported.mJointNodes = joint_nodes;
				imported.mInverseBinds = inverse_binds;
				outScene.mMeshes.push_back(std::move(imported));
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

	template <class Value>
	void AppendKey(float inTime, const Value &inValue, int inComponents, std::vector<float> &ioKeys)
	{
		ioKeys.push_back(inTime);
		const ufbx_real *value = &inValue.x;
		for (int c = 0; c < inComponents; ++c)
			ioKeys.push_back(float(value[c]));
	}

	/**
	 * 一个节点的一种通道（平移、旋转或缩放）写成 st_anim_build_animation 的键：时间和分量。只取播放范围 [0, inDuration]：
	 * 有的曲线在播放开始之前或结束之后还有键（同 Stride，范围外的不要），两端在边界上按烘焙的曲线求值补一个键。
	 */
	template <class List, class Evaluate>
	uint32_t AppendKeys(const List &inKeys, double inDuration, int inComponents, Evaluate inEvaluate, std::vector<float> &ioKeys)
	{
		if (inKeys.count == 0)
			return 0;
		// 补了边界上的键时，正好落在边界上的原有键不再要（ozz 要求时间严格递增）
		const bool clip_start = inKeys.data[0].time < 0.0;
		const bool clip_end = inKeys.data[inKeys.count - 1].time > inDuration;
		uint32_t count = 0;
		if (clip_start)
		{
			AppendKey(0.0f, inEvaluate(inKeys, 0.0), inComponents, ioKeys);
			++count;
		}
		for (size_t k = 0; k < inKeys.count; ++k)
		{
			double time = inKeys.data[k].time;
			if (clip_start ? time <= 0.0 : time < 0.0)
				continue;
			if (clip_end ? time >= inDuration : time > inDuration)
				continue;
			AppendKey(float(time), inKeys.data[k].value, inComponents, ioKeys);
			++count;
		}
		if (clip_end)
		{
			AppendKey(float(inDuration), inEvaluate(inKeys, inDuration), inComponents, ioKeys);
			++count;
		}
		return count;
	}
}

SN_API int32_t st_scene_load(const void *data, uint32_t size, int32_t format)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (data == nullptr || size == 0)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "scene data is empty");
	if (format == ST_FORMAT_AUTO)
		format = LooksLikeFbx(data, size) ? ST_FORMAT_FBX : ST_FORMAT_GLTF;
	if (format != ST_FORMAT_GLTF && format != ST_FORMAT_FBX && format != ST_FORMAT_OBJ)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "unknown scene format " + std::to_string(format));

	return RunTask("SceneLoad", [data, size, format](Task &ioTask)
	{
		auto scene = std::make_unique<ImportedScene>();
		std::string error;
		bool loaded = format == ST_FORMAT_GLTF ? ImportGltf(data, size, *scene, error) : ImportUfbx(data, size, format == ST_FORMAT_OBJ, *scene, error);
		if (!loaded)
		{
			ioTask.Fail(error);
			return false;
		}
		ioTask.SetResult(1, int32_t(scene->mMeshes.size()));
		ioTask.SetResult(2, int32_t(scene->mMaterials.size()));
		ioTask.SetResult(3, int32_t(scene->mNodes.size()));
		ioTask.SetResult(4, int32_t(scene->mAnimations.size()));
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
	out_info->node = m.mNode;
	out_info->joint_count = uint32_t(m.mJointNodes.size());
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

SN_API int32_t st_scene_node_info(int32_t scene, uint32_t node, st_node_info *out_info)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (node >= s->mNodes.size() || out_info == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "node index out of range or out_info is null");

	const ImportedNode &n = s->mNodes[node];
	out_info->parent = n.mParent;
	std::memcpy(out_info->translation, n.mTranslation, sizeof(n.mTranslation));
	std::memcpy(out_info->rotation, n.mRotation, sizeof(n.mRotation));
	std::memcpy(out_info->scale, n.mScale, sizeof(n.mScale));
	std::memcpy(out_info->world, n.mWorld, sizeof(n.mWorld));
	return SN_OK;
}

SN_API const char *st_scene_node_name(int32_t scene, uint32_t node)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return nullptr;
	if (node >= s->mNodes.size())
	{
		SetLastError(SN_ERROR_INVALID_ARGUMENT, "node index out of range");
		return nullptr;
	}
	return s->mNodes[node].mName.c_str();
}

SN_API int32_t st_scene_mesh_skin(int32_t scene, uint32_t mesh, uint32_t *joint_nodes, float *inverse_binds, uint16_t *joints, float *weights)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (mesh >= s->mMeshes.size())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "mesh index out of range");
	const ImportedMesh &m = s->mMeshes[mesh];
	if (m.mJointNodes.empty())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "mesh " + std::to_string(mesh) + " is not skinned");

	auto copy = [](auto *outTarget, const auto &inSource)
	{
		if (outTarget != nullptr && !inSource.empty())
			std::memcpy(outTarget, inSource.data(), inSource.size() * sizeof(inSource[0]));
	};
	copy(joint_nodes, m.mJointNodes);
	copy(inverse_binds, m.mInverseBinds);
	copy(joints, m.mJoints);
	copy(weights, m.mWeights);
	return SN_OK;
}

SN_API const char *st_scene_animation_name(int32_t scene, uint32_t animation)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return nullptr;
	if (animation >= s->mAnimations.size())
	{
		SetLastError(SN_ERROR_INVALID_ARGUMENT, "animation index out of range");
		return nullptr;
	}
	return s->mAnimations[animation].c_str();
}

SN_API int32_t st_scene_animation_bake(int32_t scene, uint32_t animation, float sample_rate)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (animation >= s->mAnimations.size())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "animation index out of range");
	if (s->mFbx == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "baking animations is only supported for FBX scenes");
	if (!(sample_rate > 0))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sample_rate must be positive");

	// 任务里拿着场景的共享指针：场景在任务跑完之前被销毁也没关系
	std::shared_ptr<ufbx_scene> fbx = s->mFbx;
	std::vector<const ufbx_node *> nodes = s->mFbxNodes;
	return RunTask("SceneAnimationBake", [fbx, nodes, animation, sample_rate](Task &ioTask)
	{
		ufbx_bake_opts options {};
		options.resample_rate = sample_rate;
		options.trim_start_time = true;
		ufbx_error error;
		ufbx_baked_anim *baked = ufbx_bake_anim(fbx.get(), fbx->anim_stacks.data[animation]->anim, &options, &error);
		if (baked == nullptr)
		{
			ioTask.Fail(std::string("FBX animation bake failed: ") + error.description.data);
			return false;
		}
		std::unique_ptr<ufbx_baked_anim, decltype(&ufbx_free_baked_anim)> owned(baked, &ufbx_free_baked_anim);

		std::unordered_map<uint32_t, const ufbx_baked_node *> by_node;
		for (size_t i = 0; i < baked->nodes.count; ++i)
			by_node[baked->nodes.data[i].typed_id] = &baked->nodes.data[i];

		const double duration = baked->playback_duration;
		std::vector<uint32_t> counts;
		std::vector<float> keys;
		for (const ufbx_node *node : nodes)
		{
			auto found = by_node.find(node->typed_id);
			if (found == by_node.end())
			{
				// 动画没碰的节点：一个静止的键
				const ufbx_transform &rest = node->local_transform;
				counts.insert(counts.end(), { 1, 1, 1 });
				keys.insert(keys.end(), { 0, float(rest.translation.x), float(rest.translation.y), float(rest.translation.z) });
				keys.insert(keys.end(), { 0, float(rest.rotation.x), float(rest.rotation.y), float(rest.rotation.z), float(rest.rotation.w) });
				keys.insert(keys.end(), { 0, float(rest.scale.x), float(rest.scale.y), float(rest.scale.z) });
				continue;
			}
			const ufbx_baked_node &track = *found->second;
			// trim_start_time 已经把播放开始移到 0
			counts.push_back(AppendKeys(track.translation_keys, duration, 3, ufbx_evaluate_baked_vec3, keys));
			counts.push_back(AppendKeys(track.rotation_keys, duration, 4, ufbx_evaluate_baked_quat, keys));
			counts.push_back(AppendKeys(track.scale_keys, duration, 3, ufbx_evaluate_baked_vec3, keys));
		}
		std::memcpy(ioTask.AddOutput(counts.size() * sizeof(uint32_t)), counts.data(), counts.size() * sizeof(uint32_t));
		std::memcpy(ioTask.AddOutput(keys.size() * sizeof(float)), keys.data(), keys.size() * sizeof(float));
		float stored_duration = float(duration);
		std::memcpy(ioTask.AddOutput(sizeof(float)), &stored_duration, sizeof(float));
		return true;
	});
}
