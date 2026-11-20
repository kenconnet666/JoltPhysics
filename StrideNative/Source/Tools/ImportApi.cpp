// 场景导入：glTF/GLB 用 cgltf，FBX 用 ufbx，都化成带索引的三角形网格，加上节点层级、蒙皮、形变目标和动画。
// 和 Stride 的模型导入一致：每个节点实例上的网格按材质拆开（一份材质一个网格，记下材质在场景材质表里的下标），
// 节点的世界变换烘焙进顶点和法线（镜像变换时翻转绕序），所以网格都在模型空间里，右手、Y 向上、米。
// 蒙皮网格按绑定时的姿势烘焙；每个关节的逆绑定矩阵把烘焙后的顶点变到关节空间，关节当前的节点到模型矩阵乘它就是蒙皮矩阵。
// 节点按深度优先排（父节点在子节点前面，同 Stride 的 .sdskel 和 ozz 的骨架），FBX 的根节点叫 RootNode（同 Stride 的导入）。
// 没有法线的网格补平面法线（见 AddFlatNormals）。
// 形变目标（glTF 的 targets、FBX 的 blend channel）存成每个顶点、每个目标的位置和法线增量，同样烘焙进模型空间；动画烘焙时
// 除了节点的平移、旋转、缩放，还有每个目标的权重曲线。
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
		// 形变目标：名字；增量按顶点再按目标排（[顶点][目标][xyz]，补平面法线复制顶点时整段复制），没有法线增量时 mMorphNormals 空
		std::vector<std::string> mMorphNames;
		std::vector<float>		mMorphPositions;
		std::vector<float>		mMorphNormals;
		// 驱动权重的动画通道：glTF 是挂着网格的节点（weights 通道）、FBX 是每个目标的 blend channel
		int32_t					mGltfNode = -1;
		std::vector<const ufbx_blend_channel *> mFbxChannels;
	};

	/// 留着解析好的 glTF 给动画烘焙用：GLB 的二进制块指向文件内容，所以自己存一份
	struct GltfDocument
	{
		std::vector<uint8_t>	mBytes;
		cgltf_data *			mData = nullptr;

		~GltfDocument()
		{
			cgltf_free(mData);
		}
	};

	/// 材质的贴图引用的图片：嵌在文件里的是编码过的字节（PNG、JPEG……），外部文件是相对源文件的路径
	struct ImportedImage
	{
		std::vector<uint8_t>	mBytes;
		std::string				mUri;
		std::string				mMime;
	};

	struct ImportedScene
	{
		std::vector<ImportedMesh>	mMeshes;
		std::vector<std::string>	mMaterials;		///< 材质名，按网格的 mMaterial 下标
		std::vector<st_material_info> mMaterialInfos;	///< 和 mMaterials 一一对应
		std::vector<ImportedImage>	mImages;
		std::vector<ImportedNode>	mNodes;
		std::vector<std::string>	mAnimations;
		/// 场景留着给 st_scene_animation_bake 用；mFbxNodes[i]、mGltfNodes[i] 是我们的第 i 个节点
		std::shared_ptr<ufbx_scene>	mFbx;
		std::vector<const ufbx_node *> mFbxNodes;
		std::shared_ptr<GltfDocument> mGltf;
		std::vector<const cgltf_node *> mGltfNodes;
	};

	st_texture_slot EmptySlot()
	{
		return st_texture_slot { -1, { 1.0f, 1.0f }, { 0.0f, 0.0f } };
	}

	/// glTF 的默认值（白色、不透明、金属度 1、粗糙度 1，同 glTF 规范）
	st_material_info DefaultMaterialInfo()
	{
		st_material_info info {};
		for (float &c : info.base_color)
			c = 1.0f;
		info.metallic = 1.0f;
		info.roughness = 1.0f;
		info.alpha_cutoff = 0.5f;
		info.normal_scale = 1.0f;
		info.base_color_texture = info.metallic_roughness_texture = info.normal_texture = info.occlusion_texture = info.emissive_texture = EmptySlot();
		return info;
	}

	/// data: URI 里 base64 的部分解码成字节；不是 base64 的 data URI 返回 false
	bool DecodeDataUri(const char *inUri, std::vector<uint8_t> &outBytes, std::string &outMime)
	{
		const char *comma = std::strchr(inUri, ',');
		if (comma == nullptr || std::strstr(inUri, ";base64") == nullptr || std::strstr(inUri, ";base64") > comma)
			return false;
		outMime.assign(inUri + 5, std::strcspn(inUri + 5, ";,"));
		auto value = [](char c) -> int
		{
			if (c >= 'A' && c <= 'Z') return c - 'A';
			if (c >= 'a' && c <= 'z') return c - 'a' + 26;
			if (c >= '0' && c <= '9') return c - '0' + 52;
			if (c == '+' || c == '-') return 62;
			if (c == '/' || c == '_') return 63;
			return -1;
		};
		uint32_t bits = 0;
		int count = 0;
		for (const char *c = comma + 1; *c != 0 && *c != '='; ++c)
		{
			int v = value(*c);
			if (v < 0)
				continue;
			bits = (bits << 6) | uint32_t(v);
			count += 6;
			if (count >= 8)
			{
				count -= 8;
				outBytes.push_back(uint8_t((bits >> count) & 0xff));
			}
		}
		return true;
	}

	/// 不带索引的角顶点流，用 meshoptimizer 焊接成带索引的网格。蒙皮数据也参与比较：同一个位置的角权重不同就不合并。
	struct Vertex
	{
		float		mPosition[3];
		float		mNormal[3];
		float		mUv[2];
		uint16_t	mJoints[cMaxInfluences];
		float		mWeights[cMaxInfluences];
		/// FBX 有形变目标时是控制点（mesh.vertices 的下标，形变偏移按它查），否则 0
		uint32_t	mControlPoint;
	};

	/// outControlPoints 不为空时写出每个焊接后顶点的 mControlPoint。
	ImportedMesh WeldCorners(const std::vector<Vertex> &inCorners, bool inHasNormals, bool inHasUvs, bool inSkinned, uint32_t inMaterial, std::vector<uint32_t> *outControlPoints = nullptr)
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
			if (outControlPoints != nullptr)
				outControlPoints->push_back(v.mControlPoint);
		}
		return mesh;
	}

	/// 没有法线的网格补上平面法线：Stride 导入时用 Assimp 的 aiProcess_GenNormals（逐面法线，不看 OBJ 的平滑组），glTF 规范对
	/// 缺法线的网格也要求平面着色。每个三角形用自己的面法线；一个顶点被法线不同的三角形共用时复制一份，顶点的顺序和
	/// 已有的索引不变
	void AddFlatNormals(ImportedMesh &ioMesh)
	{
		size_t vertex_count = ioMesh.mPositions.size() / 3;
		ioMesh.mNormals.assign(vertex_count * 3, 0.0f);
		// 每个原来的顶点用过的副本（第一个是它自己）
		std::vector<std::vector<uint32_t>> copies(vertex_count);
		auto copy_vertex = [&ioMesh](uint32_t inVertex) -> uint32_t
		{
			auto append = [inVertex](auto &ioValues, size_t inStride)
			{
				for (size_t i = 0; i < inStride; ++i)
					ioValues.push_back(ioValues[inVertex * inStride + i]);
			};
			uint32_t copy = uint32_t(ioMesh.mPositions.size() / 3);
			append(ioMesh.mPositions, 3);
			append(ioMesh.mNormals, 3);
			if (!ioMesh.mUvs.empty())
				append(ioMesh.mUvs, 2);
			if (!ioMesh.mJoints.empty())
			{
				append(ioMesh.mJoints, cMaxInfluences);
				append(ioMesh.mWeights, cMaxInfluences);
			}
			const size_t targets = ioMesh.mMorphNames.size();
			if (targets > 0)
				append(ioMesh.mMorphPositions, targets * 3);
			if (!ioMesh.mMorphNormals.empty())
				append(ioMesh.mMorphNormals, targets * 3);
			return copy;
		};
		for (size_t t = 0; t + 2 < ioMesh.mIndices.size(); t += 3)
		{
			const float *a = &ioMesh.mPositions[ioMesh.mIndices[t] * 3];
			const float *b = &ioMesh.mPositions[ioMesh.mIndices[t + 1] * 3];
			const float *c = &ioMesh.mPositions[ioMesh.mIndices[t + 2] * 3];
			float e1[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] }, e2[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
			float normal[3] = { e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0] };
			float length = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
			if (length > 0)
				for (float &n : normal)
					n /= length;
			for (size_t k = 0; k < 3; ++k)
			{
				uint32_t &index = ioMesh.mIndices[t + k];
				std::vector<uint32_t> &used = copies[index];
				auto same = std::find_if(used.begin(), used.end(), [&](uint32_t inCopy)
				{
					const float *n = &ioMesh.mNormals[inCopy * 3];
					return std::abs(n[0] - normal[0]) < 1.0e-5f && std::abs(n[1] - normal[1]) < 1.0e-5f && std::abs(n[2] - normal[2]) < 1.0e-5f;
				});
				if (same != used.end())
				{
					index = *same;
					continue;
				}
				uint32_t vertex = used.empty() ? index : copy_vertex(index);
				std::memcpy(&ioMesh.mNormals[vertex * 3], normal, sizeof(normal));
				used.push_back(vertex);
				index = vertex;
			}
		}
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

		/// 方向（形变的位置增量）：只用线性部分
		void Vector(const float *inV, float *outV) const
		{
			for (int r = 0; r < 3; ++r)
				outV[r] = mMatrix[r] * inV[0] + mMatrix[4 + r] * inV[1] + mMatrix[8 + r] * inV[2];
		}

		/// 法线的增量：乘法线矩阵，不归一化
		void NormalDelta(const float *inN, float *outN) const
		{
			for (int r = 0; r < 3; ++r)
				outN[r] = mNormal[r * 3] * inN[0] + mNormal[r * 3 + 1] * inN[1] + mNormal[r * 3 + 2] * inN[2];
		}

		void Normal(const float *inN, float *outN) const
		{
			NormalDelta(inN, outN);
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
		auto document = std::make_shared<GltfDocument>();
		document->mBytes.assign(static_cast<const uint8_t *>(inData), static_cast<const uint8_t *>(inData) + inSize);
		cgltf_options options {};
		if (cgltf_parse(&options, document->mBytes.data(), document->mBytes.size(), &document->mData) != cgltf_result_success)
		{
			outError = "not a valid glTF/GLB file";
			return false;
		}
		cgltf_data *data = document->mData;
		outScene.mGltf = document;
		if (cgltf_load_buffers(&options, data, nullptr) != cgltf_result_success)
		{
			outError = "glTF buffers must be embedded (GLB or data URIs)";
			return false;
		}

		for (cgltf_size i = 0; i < data->materials_count; ++i)
			outScene.mMaterials.emplace_back(data->materials[i].name != nullptr ? data->materials[i].name : "material" + std::to_string(i));
		// 图片：缓冲视图里的（GLB）、data URI 的取字节，外部文件留路径
		for (cgltf_size i = 0; i < data->images_count; ++i)
		{
			const cgltf_image &image = data->images[i];
			ImportedImage imported;
			imported.mMime = image.mime_type != nullptr ? image.mime_type : "";
			if (image.buffer_view != nullptr && image.buffer_view->buffer->data != nullptr)
			{
				const uint8_t *bytes = static_cast<const uint8_t *>(image.buffer_view->buffer->data) + image.buffer_view->offset;
				imported.mBytes.assign(bytes, bytes + image.buffer_view->size);
			}
			else if (image.uri != nullptr && std::strncmp(image.uri, "data:", 5) == 0)
			{
				std::string mime;
				if (DecodeDataUri(image.uri, imported.mBytes, mime) && imported.mMime.empty())
					imported.mMime = mime;
			}
			else if (image.uri != nullptr)
				imported.mUri = image.uri;
			outScene.mImages.push_back(std::move(imported));
		}
		auto slot_of = [&](const cgltf_texture_view &inView)
		{
			st_texture_slot slot = EmptySlot();
			// 只有 EXT_texture_webp、KHR_texture_basisu 的图（没有 PNG/JPEG 的后备）也算：编辑器、运行时能解 WebP
			const cgltf_image *image = inView.texture == nullptr ? nullptr
				: inView.texture->image != nullptr ? inView.texture->image
				: inView.texture->webp_image != nullptr ? inView.texture->webp_image : inView.texture->basisu_image;
			if (image != nullptr)
				slot.image = int32_t(image - data->images);
			if (inView.has_transform)
			{
				slot.scale[0] = inView.transform.scale[0];
				slot.scale[1] = inView.transform.scale[1];
				slot.offset[0] = inView.transform.offset[0];
				slot.offset[1] = inView.transform.offset[1];
			}
			return slot;
		};
		for (cgltf_size i = 0; i < data->materials_count; ++i)
		{
			const cgltf_material &material = data->materials[i];
			st_material_info info = DefaultMaterialInfo();
			if (material.has_pbr_metallic_roughness)
			{
				const cgltf_pbr_metallic_roughness &pbr = material.pbr_metallic_roughness;
				std::memcpy(info.base_color, pbr.base_color_factor, sizeof(info.base_color));
				info.metallic = pbr.metallic_factor;
				info.roughness = pbr.roughness_factor;
				info.base_color_texture = slot_of(pbr.base_color_texture);
				info.metallic_roughness_texture = slot_of(pbr.metallic_roughness_texture);
			}
			float strength = material.has_emissive_strength ? material.emissive_strength.emissive_strength : 1.0f;
			for (int c = 0; c < 3; ++c)
				info.emissive[c] = material.emissive_factor[c] * strength;
			info.emissive_texture = slot_of(material.emissive_texture);
			info.normal_texture = slot_of(material.normal_texture);
			info.normal_scale = material.normal_texture.texture != nullptr ? material.normal_texture.scale : 1.0f;
			info.occlusion_texture = slot_of(material.occlusion_texture);
			info.alpha_mode = material.alpha_mode == cgltf_alpha_mode_mask ? 1 : material.alpha_mode == cgltf_alpha_mode_blend ? 2 : 0;
			info.alpha_cutoff = material.alpha_cutoff;
			info.double_sided = material.double_sided ? 1 : 0;
			outScene.mMaterialInfos.push_back(info);
		}
		for (cgltf_size i = 0; i < data->animations_count; ++i)
			outScene.mAnimations.emplace_back(data->animations[i].name != nullptr ? data->animations[i].name : "animation" + std::to_string(i));

		// 节点按深度优先排：从没有父节点的节点开始
		std::vector<uint32_t> index_of(data->nodes_count, 0);
		auto add_node = [&](auto &&inSelf, const cgltf_node &inNode, int32_t inParent) -> void
		{
			uint32_t index = uint32_t(outScene.mNodes.size());
			index_of[size_t(&inNode - data->nodes)] = index;
			outScene.mGltfNodes.push_back(&inNode);
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

		auto import_mesh = [&](const cgltf_mesh &inMesh, const NodeTransform &inTransform, uint32_t inNode, int32_t inGltfNode, const cgltf_skin *inSkin)
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
				if (primitive.targets_count > 0)
				{
					const size_t targets = primitive.targets_count;
					mesh.mGltfNode = inGltfNode;
					mesh.mMorphPositions.assign(positions->count * targets * 3, 0.0f);
					for (size_t t = 0; t < targets; ++t)
					{
						mesh.mMorphNames.emplace_back(t < inMesh.target_names_count && inMesh.target_names[t] != nullptr ? inMesh.target_names[t] : "target" + std::to_string(t));
						const cgltf_morph_target &target = primitive.targets[t];
						for (cgltf_size a = 0; a < target.attributes_count; ++a)
						{
							const cgltf_attribute &attribute = target.attributes[a];
							bool is_normal = attribute.type == cgltf_attribute_type_normal;
							if ((attribute.type != cgltf_attribute_type_position && !is_normal) || attribute.data->count != positions->count)
								continue;
							if (is_normal && mesh.mMorphNormals.empty())
								mesh.mMorphNormals.assign(positions->count * targets * 3, 0.0f);
							local.resize(attribute.data->count * 3);
							cgltf_accessor_unpack_floats(attribute.data, local.data(), local.size());
							std::vector<float> &deltas = is_normal ? mesh.mMorphNormals : mesh.mMorphPositions;
							for (size_t v = 0; v < positions->count; ++v)
							{
								float *out = &deltas[(v * targets + t) * 3];
								if (is_normal)
									inTransform.NormalDelta(&local[v * 3], out);
								else
									inTransform.Vector(&local[v * 3], out);
							}
						}
					}
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
				if (mesh.mNormals.empty())
					AddFlatNormals(mesh);
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
			import_mesh(*node.mesh, NodeTransform(node.skin != nullptr ? identity : world), index_of[n], int32_t(n), node.skin);
		}
		if (!any_node)
			for (cgltf_size m = 0; m < data->meshes_count; ++m)
				import_mesh(data->meshes[m], NodeTransform(identity), 0, -1, nullptr);
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

	/// 每个焊接后的顶点按它的控制点取各个目标形状的偏移，变到模型空间
	void AddFbxMorphTargets(const std::vector<const ufbx_blend_channel *> &inChannels, const std::vector<uint32_t> &inControlPoints, const ufbx_matrix &inToWorld,
		const ufbx_matrix &inToWorldNormals, ImportedMesh &ioMesh)
	{
		const size_t targets = inChannels.size();
		const size_t vertex_count = inControlPoints.size();
		ioMesh.mFbxChannels = inChannels;
		bool has_normals = std::any_of(inChannels.begin(), inChannels.end(), [](const ufbx_blend_channel *inChannel) { return inChannel->target_shape->normal_offsets.count > 0; });
		ioMesh.mMorphPositions.assign(vertex_count * targets * 3, 0.0f);
		if (has_normals)
			ioMesh.mMorphNormals.assign(vertex_count * targets * 3, 0.0f);
		for (size_t t = 0; t < targets; ++t)
		{
			const ufbx_blend_channel &channel = *inChannels[t];
			const ufbx_blend_shape &shape = *channel.target_shape;
			ioMesh.mMorphNames.emplace_back(channel.name.length > 0 ? channel.name.data : shape.name.data);
			for (size_t v = 0; v < vertex_count; ++v)
			{
				uint32_t offset = ufbx_get_blend_shape_offset_index(&shape, inControlPoints[v]);
				if (offset == UFBX_NO_INDEX)
					continue;
				auto store = [&](const ufbx_vec3 &inValue, std::vector<float> &outDeltas)
				{
					float *out = &outDeltas[(v * targets + t) * 3];
					out[0] = float(inValue.x); out[1] = float(inValue.y); out[2] = float(inValue.z);
				};
				store(ufbx_transform_direction(&inToWorld, shape.position_offsets.data[offset]), ioMesh.mMorphPositions);
				if (has_normals && offset < shape.normal_offsets.count)
					store(ufbx_transform_direction(&inToWorldNormals, shape.normal_offsets.data[offset]), ioMesh.mMorphNormals);
			}
		}
	}

	/// FBX 和 OBJ 都由 ufbx 读；OBJ 不带坐标轴和单位，按右手 Y 向上、米（不做转换）
	bool ImportUfbx(const void *inData, uint32_t inSize, bool inObj, ImportedScene &outScene, std::string &outError)
	{
		ufbx_load_opts options {};
		options.target_axes = ufbx_axes_right_handed_y_up;
		options.target_unit_meters = 1.0f;
		options.space_conversion = UFBX_SPACE_CONVERSION_MODIFY_GEOMETRY;
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
		// 图片：每个文件贴图一张（ufbx 的 typed_id 换成我们的下标）；嵌在 FBX 里的取字节，外部文件留相对路径
		std::unordered_map<uint32_t, int32_t> image_of;
		for (size_t i = 0; i < scene->textures.count; ++i)
		{
			const ufbx_texture &texture = *scene->textures.data[i];
			if (texture.type != UFBX_TEXTURE_FILE)
				continue;
			ImportedImage imported;
			if (texture.content.size > 0)
			{
				const uint8_t *bytes = static_cast<const uint8_t *>(texture.content.data);
				imported.mBytes.assign(bytes, bytes + texture.content.size);
			}
			else
				imported.mUri = texture.relative_filename.data;
			image_of[texture.typed_id] = int32_t(outScene.mImages.size());
			outScene.mImages.push_back(std::move(imported));
		}
		auto fbx_slot = [&](const ufbx_material_map &inMap)
		{
			st_texture_slot slot = EmptySlot();
			const ufbx_texture *texture = inMap.texture_enabled ? inMap.texture : nullptr;
			// 贴图是节点图、多层的时候取第一张文件贴图（ufbx 的 file_textures）
			if (texture != nullptr && texture->type != UFBX_TEXTURE_FILE)
				texture = texture->file_textures.count > 0 ? texture->file_textures.data[0] : nullptr;
			if (texture == nullptr)
				return slot;
			auto found = image_of.find(texture->typed_id);
			if (found != image_of.end())
				slot.image = found->second;
			if (texture->has_uv_transform)
			{
				slot.scale[0] = float(texture->uv_transform.scale.x);
				slot.scale[1] = float(texture->uv_transform.scale.y);
				slot.offset[0] = float(texture->uv_transform.translation.x);
				slot.offset[1] = float(texture->uv_transform.translation.y);
			}
			return slot;
		};
		for (size_t i = 0; i < scene->materials.count; ++i)
		{
			const ufbx_material &material = *scene->materials.data[i];
			st_material_info info = DefaultMaterialInfo();
			const ufbx_vec4 &base = material.pbr.base_color.value_vec4;
			float factor = material.pbr.base_factor.has_value ? float(material.pbr.base_factor.value_real) : 1.0f;
			info.base_color[0] = float(base.x) * factor;
			info.base_color[1] = float(base.y) * factor;
			info.base_color[2] = float(base.z) * factor;
			info.base_color[3] = material.pbr.opacity.has_value ? float(material.pbr.opacity.value_real) : 1.0f;
			// 没写金属度的（FBX 的 Phong、Lambert）当非金属
			info.metallic = material.pbr.metalness.has_value ? float(material.pbr.metalness.value_real) : 0.0f;
			info.roughness = material.pbr.roughness.has_value ? float(material.pbr.roughness.value_real) : 0.5f;
			float emission = material.pbr.emission_factor.has_value ? float(material.pbr.emission_factor.value_real) : 1.0f;
			info.emissive[0] = float(material.pbr.emission_color.value_vec3.x) * emission;
			info.emissive[1] = float(material.pbr.emission_color.value_vec3.y) * emission;
			info.emissive[2] = float(material.pbr.emission_color.value_vec3.z) * emission;
			info.base_color_texture = fbx_slot(material.pbr.base_color);
			info.normal_texture = fbx_slot(material.pbr.normal_map);
			info.occlusion_texture = fbx_slot(material.pbr.ambient_occlusion);
			info.emissive_texture = fbx_slot(material.pbr.emission_color);
			info.alpha_mode = info.base_color[3] < 1.0f ? 2 : 0;
			info.double_sided = material.features.double_sided.enabled ? 1 : 0;
			outScene.mMaterialInfos.push_back(info);
		}
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
			// 形变目标：每个有目标形状的 blend channel 一个（同 Assimp，Stride 的导入用它）
			std::vector<const ufbx_blend_channel *> channels;
			for (size_t d = 0; d < mesh.blend_deformers.count; ++d)
				for (size_t c = 0; c < mesh.blend_deformers.data[d]->channels.count; ++c)
					if (const ufbx_blend_channel *channel = mesh.blend_deformers.data[d]->channels.data[c]; channel->target_shape != nullptr)
						channels.push_back(channel);
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
						if (!channels.empty())
							vertex.mControlPoint = mesh.vertex_indices.data[index];
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
				std::vector<uint32_t> control_points;
				ImportedMesh imported = WeldCorners(corners, mesh.vertex_normal.exists, mesh.vertex_uv.exists, skin != nullptr, material_index, &control_points);
				if (!channels.empty())
					AddFbxMorphTargets(channels, control_points, to_world, to_world_normals, imported);
				if (!mesh.vertex_normal.exists)
					AddFlatNormals(imported);
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

	/// 任务的一块输出：空的不写（AddOutput(0) 没有内存）
	template <class T>
	void PutOutput(Task &ioTask, const std::vector<T> &inValues)
	{
		void *out = ioTask.AddOutput(uint32_t(inValues.size() * sizeof(T)));
		if (!inValues.empty())
			std::memcpy(out, inValues.data(), inValues.size() * sizeof(T));
	}

	/// 两个单位四元数（xyzw）的球面插值，走短的那边
	void Slerp(const float *inA, const float *inB, float inT, float *outQ)
	{
		float b[4] = { inB[0], inB[1], inB[2], inB[3] };
		float cosine = inA[0] * b[0] + inA[1] * b[1] + inA[2] * b[2] + inA[3] * b[3];
		if (cosine < 0)
		{
			cosine = -cosine;
			for (float &c : b)
				c = -c;
		}
		float wa = 1 - inT, wb = inT;
		if (cosine < 0.9995f)
		{
			float angle = std::acos(cosine), sine = std::sin(angle);
			wa = std::sin((1 - inT) * angle) / sine;
			wb = std::sin(inT * angle) / sine;
		}
		float length = 0;
		for (int i = 0; i < 4; ++i)
		{
			outQ[i] = wa * inA[i] + wb * b[i];
			length += outQ[i] * outQ[i];
		}
		length = std::sqrt(length);
		if (length > 0)
			for (int i = 0; i < 4; ++i)
				outQ[i] /= length;
	}

	/// glTF 动画的一个采样器：输入时间、输出值读出来，按插值方式（STEP、LINEAR、CUBICSPLINE）求任意时间的值。
	/// mWidth 是一个值几个数（平移 3、旋转 4、权重是目标数）；三次样条每个键存入切线、值、出切线三组。
	struct GltfSampler
	{
		std::vector<float>			mTimes;
		std::vector<float>			mValues;
		int							mWidth;
		cgltf_interpolation_type	mInterpolation;
		bool						mRotation;

		GltfSampler(const cgltf_animation_sampler &inSampler, int inWidth, bool inRotation) :
			mWidth(inWidth), mInterpolation(inSampler.interpolation), mRotation(inRotation)
		{
			mTimes.resize(inSampler.input->count);
			cgltf_accessor_unpack_floats(inSampler.input, mTimes.data(), mTimes.size());
			mValues.resize(cgltf_accessor_unpack_floats(inSampler.output, nullptr, 0));
			cgltf_accessor_unpack_floats(inSampler.output, mValues.data(), mValues.size());
		}

		float Duration() const
		{
			return mTimes.empty() ? 0.0f : mTimes.back();
		}

		void Evaluate(float inTime, float *outValue) const
		{
			const bool cubic = mInterpolation == cgltf_interpolation_type_cubic_spline;
			const size_t stride = size_t(cubic ? 3 * mWidth : mWidth), value_offset = size_t(cubic ? mWidth : 0);
			const size_t count = std::min(mTimes.size(), mValues.size() / stride);
			auto value = [&](size_t inKey) { return &mValues[inKey * stride + value_offset]; };
			if (count == 0)
				return;
			if (count == 1 || inTime <= mTimes[0] || inTime >= mTimes[count - 1])
			{
				std::memcpy(outValue, value(inTime <= mTimes[0] ? 0 : count - 1), sizeof(float) * mWidth);
				return;
			}
			size_t k = size_t(std::upper_bound(mTimes.begin(), mTimes.begin() + count, inTime) - mTimes.begin()) - 1;
			const float dt = mTimes[k + 1] - mTimes[k], u = dt > 0 ? (inTime - mTimes[k]) / dt : 0.0f;
			if (mInterpolation == cgltf_interpolation_type_step)
				std::memcpy(outValue, value(k), sizeof(float) * mWidth);
			else if (!cubic && mRotation)
				Slerp(value(k), value(k + 1), u, outValue);
			else if (!cubic)
				for (int i = 0; i < mWidth; ++i)
					outValue[i] = value(k)[i] + (value(k + 1)[i] - value(k)[i]) * u;
			else
			{
				// Hermite：值 p、切线 m（切线乘键间隔）
				const float u2 = u * u, u3 = u2 * u;
				const float h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
				const float *out_tangent = &mValues[k * stride + 2 * mWidth], *in_tangent = &mValues[(k + 1) * stride];
				for (int i = 0; i < mWidth; ++i)
					outValue[i] = h00 * value(k)[i] + h10 * out_tangent[i] * dt + h01 * value(k + 1)[i] + h11 * in_tangent[i] * dt;
				if (mRotation)
				{
					float length = std::sqrt(outValue[0] * outValue[0] + outValue[1] * outValue[1] + outValue[2] * outValue[2] + outValue[3] * outValue[3]);
					if (length > 0)
						for (int i = 0; i < 4; ++i)
							outValue[i] /= length;
				}
			}
		}
	};

	/// 形变网格的动画来源：glTF 是挂着它的节点，FBX 是每个目标的 blend channel
	struct MorphSource
	{
		uint32_t								mTargets;
		int32_t									mGltfNode;
		std::vector<const ufbx_blend_channel *>	mFbxChannels;
	};

	std::vector<MorphSource> MorphSources(const ImportedScene &inScene)
	{
		std::vector<MorphSource> sources;
		for (const ImportedMesh &mesh : inScene.mMeshes)
			if (!mesh.mMorphNames.empty())
				sources.push_back({ uint32_t(mesh.mMorphNames.size()), mesh.mGltfNode, mesh.mFbxChannels });
		return sources;
	}

	/**
	 * 烘焙 glTF 的动画：按 inSampleRate 重采样（最后补一个正好在结尾的键），输出和 FBX 的一样。动画没碰的节点一个静止的键；
	 * 旋转相邻的键取同一半球（ozz 按分量插值）。
	 */
	void BakeGltfAnimation(const cgltf_animation &inAnimation, const cgltf_data &inData, const std::vector<const cgltf_node *> &inNodes,
		const std::vector<ImportedNode> &inRest, const std::vector<MorphSource> &inMorphs, float inSampleRate, Task &ioTask)
	{
		// 每个节点、每种通道（0 平移、1 旋转、2 缩放、3 权重）的采样器
		std::unordered_map<size_t, GltfSampler> samplers;
		float duration = 0;
		for (cgltf_size c = 0; c < inAnimation.channels_count; ++c)
		{
			const cgltf_animation_channel &channel = inAnimation.channels[c];
			if (channel.target_node == nullptr || channel.sampler == nullptr)
				continue;
			int path = channel.target_path == cgltf_animation_path_type_translation ? 0 : channel.target_path == cgltf_animation_path_type_rotation ? 1
				: channel.target_path == cgltf_animation_path_type_scale ? 2 : channel.target_path == cgltf_animation_path_type_weights ? 3 : -1;
			if (path < 0)
				continue;
			const cgltf_node &node = *channel.target_node;
			int width = path == 1 ? 4 : path == 3 ? int(node.mesh != nullptr ? std::max<cgltf_size>(node.mesh->weights_count, node.mesh->primitives_count > 0 ? node.mesh->primitives[0].targets_count : 0) : 0) : 3;
			if (width == 0)
				continue;
			GltfSampler &sampler = samplers.try_emplace(size_t(&node - inData.nodes) * 4 + size_t(path), *channel.sampler, width, path == 1).first->second;
			duration = std::max(duration, sampler.Duration());
		}

		std::vector<float> times;
		for (uint32_t i = 0; float(i) / inSampleRate < duration; ++i)
			times.push_back(float(i) / inSampleRate);
		times.push_back(duration);

		auto find = [&](size_t inNode, int inPath) -> const GltfSampler *
		{
			auto found = samplers.find(inNode * 4 + size_t(inPath));
			return found != samplers.end() ? &found->second : nullptr;
		};
		std::vector<uint32_t> counts;
		std::vector<float> keys;
		std::vector<float> value(16);
		for (size_t n = 0; n < inNodes.size(); ++n)
		{
			const size_t gltf_node = size_t(inNodes[n] - inData.nodes);
			const ImportedNode &rest = inRest[n];
			const float *rest_values[3] = { rest.mTranslation, rest.mRotation, rest.mScale };
			for (int path = 0; path < 3; ++path)
			{
				const int width = path == 1 ? 4 : 3;
				const GltfSampler *sampler = find(gltf_node, path);
				if (sampler == nullptr)
				{
					counts.push_back(1);
					keys.push_back(0);
					keys.insert(keys.end(), rest_values[path], rest_values[path] + width);
					continue;
				}
				counts.push_back(uint32_t(times.size()));
				const size_t first = keys.size();
				for (size_t i = 0; i < times.size(); ++i)
				{
					sampler->Evaluate(times[i], value.data());
					if (path == 1 && i > 0)
					{
						const float *previous = &keys[first + (i - 1) * 5 + 1];
						if (previous[0] * value[0] + previous[1] * value[1] + previous[2] * value[2] + previous[3] * value[3] < 0)
							for (int c = 0; c < 4; ++c)
								value[c] = -value[c];
					}
					keys.push_back(times[i]);
					keys.insert(keys.end(), value.begin(), value.begin() + width);
				}
			}
		}

		std::vector<uint32_t> weight_counts;
		std::vector<float> weight_keys;
		for (const MorphSource &morph : inMorphs)
		{
			const GltfSampler *sampler = morph.mGltfNode >= 0 ? find(size_t(morph.mGltfNode), 3) : nullptr;
			if (sampler == nullptr || uint32_t(sampler->mWidth) < morph.mTargets)
			{
				weight_counts.insert(weight_counts.end(), morph.mTargets, 0);
				continue;
			}
			value.resize(std::max<size_t>(value.size(), size_t(sampler->mWidth)));
			std::vector<std::vector<float>> per_target(morph.mTargets);
			for (float time : times)
			{
				sampler->Evaluate(time, value.data());
				for (uint32_t t = 0; t < morph.mTargets; ++t)
					per_target[t].insert(per_target[t].end(), { time, value[t] });
			}
			for (const std::vector<float> &target : per_target)
			{
				weight_counts.push_back(uint32_t(times.size()));
				weight_keys.insert(weight_keys.end(), target.begin(), target.end());
			}
		}

		PutOutput(ioTask, counts);
		PutOutput(ioTask, keys);
		PutOutput(ioTask, std::vector<float> { duration });
		PutOutput(ioTask, weight_counts);
		PutOutput(ioTask, weight_keys);
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
	out_info->morph_target_count = uint32_t(m.mMorphNames.size());
	out_info->morph_has_normals = !m.mMorphNormals.empty();
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

SN_API int32_t st_scene_material_info(int32_t scene, uint32_t material, st_material_info *out_info)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (out_info == nullptr || material >= s->mMaterialInfos.size())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "material index out of range (or out_info is null)");
	*out_info = s->mMaterialInfos[material];
	return SN_OK;
}

SN_API int32_t st_scene_image_count(int32_t scene)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	return int32_t(s->mImages.size());
}

SN_API int32_t st_scene_image(int32_t scene, uint32_t image, const uint8_t **out_data, const char **out_uri, const char **out_mime)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (image >= s->mImages.size() || out_data == nullptr || out_uri == nullptr || out_mime == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "image index out of range (or an output is null)");
	const ImportedImage &imported = s->mImages[image];
	*out_data = imported.mBytes.data();
	*out_uri = imported.mUri.c_str();
	*out_mime = imported.mMime.c_str();
	return int32_t(imported.mBytes.size());
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

SN_API const char *st_scene_mesh_morph_name(int32_t scene, uint32_t mesh, uint32_t target)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return nullptr;
	if (mesh >= s->mMeshes.size() || target >= s->mMeshes[mesh].mMorphNames.size())
	{
		SetLastError(SN_ERROR_INVALID_ARGUMENT, "mesh or morph target index out of range");
		return nullptr;
	}
	return s->mMeshes[mesh].mMorphNames[target].c_str();
}

SN_API int32_t st_scene_mesh_morph(int32_t scene, uint32_t mesh, uint32_t target, float *position_deltas, float *normal_deltas)
{
	ImportedScene *s = Lookup<ImportedScene>(scene, "scene");
	if (s == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (mesh >= s->mMeshes.size() || target >= s->mMeshes[mesh].mMorphNames.size())
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "mesh or morph target index out of range");

	const ImportedMesh &m = s->mMeshes[mesh];
	const size_t targets = m.mMorphNames.size(), vertex_count = m.mPositions.size() / 3;
	auto copy = [&](float *outDeltas, const std::vector<float> &inDeltas)
	{
		if (outDeltas == nullptr || inDeltas.empty())
			return;
		for (size_t v = 0; v < vertex_count; ++v)
			std::memcpy(&outDeltas[v * 3], &inDeltas[(v * targets + target) * 3], 3 * sizeof(float));
	};
	copy(position_deltas, m.mMorphPositions);
	copy(normal_deltas, m.mMorphNormals);
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
	if (!(sample_rate > 0))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "sample_rate must be positive");
	std::vector<MorphSource> morphs = MorphSources(*s);
	if (s->mGltf != nullptr)
	{
		std::shared_ptr<GltfDocument> gltf = s->mGltf;
		std::vector<const cgltf_node *> nodes = s->mGltfNodes;
		std::vector<ImportedNode> rest = s->mNodes;
		return RunTask("SceneAnimationBake", [gltf, nodes, rest, morphs, animation, sample_rate](Task &ioTask)
		{
			BakeGltfAnimation(gltf->mData->animations[animation], *gltf->mData, nodes, rest, morphs, sample_rate, ioTask);
			return true;
		});
	}

	// 任务里拿着场景的共享指针：场景在任务跑完之前被销毁也没关系
	std::shared_ptr<ufbx_scene> fbx = s->mFbx;
	std::vector<const ufbx_node *> nodes = s->mFbxNodes;
	return RunTask("SceneAnimationBake", [fbx, nodes, morphs, animation, sample_rate](Task &ioTask)
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

		// 形变权重：blend channel 的 DeformPercent（0 到 100）
		std::vector<uint32_t> weight_counts;
		std::vector<float> weight_keys;
		for (const MorphSource &morph : morphs)
			for (const ufbx_blend_channel *channel : morph.mFbxChannels)
			{
				const ufbx_baked_element *element = ufbx_find_baked_element_by_element_id(baked, channel->element_id);
				const ufbx_baked_prop *percent = nullptr;
				for (size_t p = 0; element != nullptr && p < element->props.count; ++p)
					if (std::strcmp(element->props.data[p].name.data, "DeformPercent") == 0)
						percent = &element->props.data[p];
				if (percent == nullptr)
				{
					weight_counts.push_back(0);
					continue;
				}
				const size_t first = weight_keys.size();
				weight_counts.push_back(AppendKeys(percent->keys, duration, 1, ufbx_evaluate_baked_vec3, weight_keys));
				for (size_t k = first + 1; k < weight_keys.size(); k += 2)
					weight_keys[k] /= 100.0f;
			}

		PutOutput(ioTask, counts);
		PutOutput(ioTask, keys);
		PutOutput(ioTask, std::vector<float> { float(duration) });
		PutOutput(ioTask, weight_counts);
		PutOutput(ioTask, weight_keys);
		return true;
	});
}
