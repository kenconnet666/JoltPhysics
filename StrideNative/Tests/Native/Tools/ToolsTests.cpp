#include <TestSupport.h>
#include <StrideNative/Tools.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace
{
	std::vector<char> ReadFile(const std::string &inPath)
	{
		std::ifstream file(inPath, std::ios::binary);
		return std::vector<char>(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
	}

	std::string TestDataPath(const char *inName)
	{
		return std::string(STRIDE_NATIVE_TEST_DATA) + "/" + inName;
	}

	/// Unit cube, 24 vertices (split per face) so normals and uvs are per face.
	struct Cube
	{
		std::vector<float> mPositions, mNormals, mUvs;
		std::vector<uint32_t> mIndices;

		Cube()
		{
			const float faces[6][3] = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
			for (int f = 0; f < 6; ++f)
			{
				const float *n = faces[f];
				// Two axes spanning the face: u is any axis perpendicular to n, v = n x u.
				float u[3] = { n[0] != 0 ? 0.0f : 1.0f, 0, n[0] != 0 ? 1.0f : 0.0f }, v[3];
				v[0] = n[1] * u[2] - n[2] * u[1];
				v[1] = n[2] * u[0] - n[0] * u[2];
				v[2] = n[0] * u[1] - n[1] * u[0];
				uint32_t base = uint32_t(mPositions.size() / 3);
				const float corners[4][2] = { { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } };
				for (const auto &c : corners)
				{
					for (int axis = 0; axis < 3; ++axis)
						mPositions.push_back(0.5f * (n[axis] + c[0] * u[axis] + c[1] * v[axis]));
					mNormals.insert(mNormals.end(), n, n + 3);
					mUvs.insert(mUvs.end(), { (c[0] + 1) / 2, (c[1] + 1) / 2 });
				}
				mIndices.insert(mIndices.end(), { base, base + 1, base + 2, base, base + 2, base + 3 });
			}
		}
	};
}

TEST_CASE("FBX and glTF scenes import as indexed meshes")
{
	ModuleScope module;
	std::vector<char> fbx = ReadFile(TestDataPath("cube.fbx"));
	REQUIRE(!fbx.empty());
	int32_t task = st_scene_load(fbx.data(), uint32_t(fbx.size()), ST_FORMAT_AUTO);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	int32_t scene = sn_task_result(task, 0);
	REQUIRE(sn_task_result(task, 1) == 1);
	sn_task_release(task);

	st_mesh_info info;
	REQUIRE(st_scene_mesh_info(scene, 0, &info) == SN_OK);
	CHECK(info.index_count == 36);
	CHECK(info.vertex_count == 24);
	CHECK(info.has_normals);
	st_scene_destroy(scene);

	// Minimal glTF: one triangle in an embedded base64 buffer (3 float3 positions = 36 bytes).
	const std::string gltf = R"({"asset":{"version":"2.0"},
		"buffers":[{"byteLength":36,"uri":"data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAA"}],
		"bufferViews":[{"buffer":0,"byteLength":36}],
		"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]}],
		"meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}]})";
	task = st_scene_load(gltf.data(), uint32_t(gltf.size()), ST_FORMAT_AUTO);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	scene = sn_task_result(task, 0);
	sn_task_release(task);
	REQUIRE(st_scene_mesh_info(scene, 0, &info) == SN_OK);
	CHECK(info.vertex_count == 3);
	std::vector<float> positions(9);
	REQUIRE(st_scene_mesh_copy(scene, 0, positions.data(), nullptr, nullptr, nullptr) == SN_OK);
	CHECK(positions[3] == 1.0f);
	CHECK(info.material == ST_NO_MATERIAL);
	st_scene_destroy(scene);
}

TEST_CASE("场景导入：每个节点实例一个网格，节点变换烘焙进顶点，镜像时翻转绕序，记下材质")
{
	ModuleScope module;
	// 同一个三角形网格挂在两个节点上：一个平移 (10, 0, 0)，一个沿 X 镜像；三角形用材质 Grid
	const std::string gltf = R"({"asset":{"version":"2.0"},
		"buffers":[{"byteLength":36,"uri":"data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAA"}],
		"bufferViews":[{"buffer":0,"byteLength":36}],
		"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]}],
		"materials":[{"name":"Grid"}],
		"meshes":[{"primitives":[{"attributes":{"POSITION":0},"material":0}]}],
		"nodes":[{"mesh":0,"translation":[10,0,0]},{"mesh":0,"scale":[-1,1,1]}],
		"scenes":[{"nodes":[0,1]}]})";
	int32_t task = st_scene_load(gltf.data(), uint32_t(gltf.size()), ST_FORMAT_GLTF);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	int32_t scene = sn_task_result(task, 0);
	CHECK(sn_task_result(task, 1) == 2);
	CHECK(sn_task_result(task, 2) == 1);
	sn_task_release(task);
	CHECK(std::string(st_scene_material_name(scene, 0)) == "Grid");
	CHECK(st_scene_material_name(scene, 1) == nullptr);

	st_mesh_info info;
	std::vector<float> positions(9);
	std::vector<uint32_t> indices(3);
	REQUIRE(st_scene_mesh_info(scene, 0, &info) == SN_OK);
	CHECK(info.material == 0);
	REQUIRE(st_scene_mesh_copy(scene, 0, positions.data(), nullptr, nullptr, indices.data()) == SN_OK);
	CHECK(positions[0] == 10.0f);	// 第一个顶点 (0, 0, 0) 平移到 (10, 0, 0)
	CHECK(positions[3] == 11.0f);
	CHECK((indices[0] == 0 && indices[1] == 1 && indices[2] == 2));

	REQUIRE(st_scene_mesh_copy(scene, 1, positions.data(), nullptr, nullptr, indices.data()) == SN_OK);
	CHECK(positions[3] == -1.0f);	// (1, 0, 0) 镜像到 (-1, 0, 0)
	CHECK((indices[0] == 0 && indices[1] == 2 && indices[2] == 1));	// 绕序翻转，正面仍朝外
	st_scene_destroy(scene);
}

TEST_CASE("场景导入：节点按深度优先排，蒙皮网格带关节、逆绑定矩阵和按权重排好的影响，不受自己节点的变换影响")
{
	ModuleScope module;
	// root → tip（在 (0, 1, 0)）；body 挂着一个三角形，蒙皮到 root 和 tip；body 自己的平移 (5, 0, 0) 按 glTF 规范不作用于蒙皮网格
	const std::string gltf = R"({"asset":{"version":"2.0"},
		"buffers":[{"byteLength":224,"uri":"data:application/octet-stream;base64,AAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAAAAEAAAEAAAABAAAAAACAPgAAQD8AAAAAAAAAAAAAgD8AAAAAAAAAAAAAAAAAAAA/AAAAPwAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAAAAAAIA/AAAAAAAAAAAAAAAAAAAAAAAAgD8AAAAAAAAAAAAAAAAAAAAAAACAPwAAgD8AAAAAAAAAAAAAAAAAAAAAAACAPwAAAAAAAAAAAAAAAAAAAAAAAIA/AAAAAAAAAAAAAIC/AAAAAAAAgD8="}],
		"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":12},{"buffer":0,"byteOffset":48,"byteLength":48},{"buffer":0,"byteOffset":96,"byteLength":128}],
		"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]},
			{"bufferView":1,"componentType":5121,"count":3,"type":"VEC4"},
			{"bufferView":2,"componentType":5126,"count":3,"type":"VEC4"},
			{"bufferView":3,"componentType":5126,"count":2,"type":"MAT4"}],
		"meshes":[{"primitives":[{"attributes":{"POSITION":0,"JOINTS_0":1,"WEIGHTS_0":2}}]}],
		"skins":[{"joints":[0,1],"inverseBindMatrices":3}],
		"nodes":[{"name":"root","children":[1]},{"name":"tip","translation":[0,1,0]},{"name":"body","mesh":0,"skin":0,"translation":[5,0,0]}],
		"scenes":[{"nodes":[0,2]}]})";
	int32_t task = st_scene_load(gltf.data(), uint32_t(gltf.size()), ST_FORMAT_GLTF);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	int32_t scene = sn_task_result(task, 0);
	CHECK(sn_task_result(task, 3) == 3);
	sn_task_release(task);

	st_node_info node;
	REQUIRE(st_scene_node_info(scene, 1, &node) == SN_OK);
	CHECK(std::string(st_scene_node_name(scene, 1)) == "tip");
	CHECK(node.parent == 0);
	CHECK(node.translation[1] == 1.0f);
	CHECK(node.world[13] == 1.0f);
	CHECK(std::string(st_scene_node_name(scene, 2)) == "body");

	st_mesh_info info;
	REQUIRE(st_scene_mesh_info(scene, 0, &info) == SN_OK);
	CHECK(info.node == 2);
	REQUIRE(info.joint_count == 2);
	std::vector<float> positions(9), inverse_binds(32), weights(12);
	std::vector<uint32_t> joint_nodes(2);
	std::vector<uint16_t> joints(12);
	REQUIRE(st_scene_mesh_copy(scene, 0, positions.data(), nullptr, nullptr, nullptr) == SN_OK);
	CHECK(positions[3] == 1.0f);	// 没有加上 body 的平移
	REQUIRE(st_scene_mesh_skin(scene, 0, joint_nodes.data(), inverse_binds.data(), joints.data(), weights.data()) == SN_OK);
	CHECK((joint_nodes[0] == 0 && joint_nodes[1] == 1));
	CHECK(inverse_binds[16 + 13] == -1.0f);	// tip 的逆绑定：平移 (0, -1, 0)
	// 第一个顶点：tip 0.75、root 0.25，按权重从大到小
	CHECK((joints[0] == 1 && joints[1] == 0));
	CHECK((weights[0] == 0.75f && weights[1] == 0.25f && weights[2] == 0.0f));
	CHECK(st_scene_animation_bake(scene, 0, 30) < 0);	// glTF 的动画还不支持
	st_scene_destroy(scene);
}

TEST_CASE("场景导入：STRIDE_TEST_SCENE 指定的文件（排查用，没设时跳过）")
{
	const char *file = std::getenv("STRIDE_TEST_SCENE");
	if (file == nullptr)
		return;
	ModuleScope module;
	std::vector<char> data = ReadFile(file);
	REQUIRE(!data.empty());
	int32_t task = st_scene_load(data.data(), uint32_t(data.size()), ST_FORMAT_AUTO);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	int32_t scene = sn_task_result(task, 0);
	MESSAGE("meshes " << sn_task_result(task, 1) << ", nodes " << sn_task_result(task, 3) << ", animations " << sn_task_result(task, 4));
	sn_task_release(task);
	if (st_scene_animation_name(scene, 0) != nullptr)
	{
		int32_t bake = st_scene_animation_bake(scene, 0, 30);
		REQUIRE(WaitForTask(bake) == SN_TASK_DONE);
		sn_task_release(bake);
	}
	st_scene_destroy(scene);
}

TEST_CASE("Tangents, meshopt, UV unwrap and convex decomposition")
{
	ModuleScope module;
	Cube cube;
	uint32_t vertex_count = uint32_t(cube.mPositions.size() / 3), index_count = uint32_t(cube.mIndices.size());

	std::vector<float> tangents(index_count * 4);
	REQUIRE(st_generate_tangents(cube.mPositions.data(), cube.mNormals.data(), cube.mUvs.data(), cube.mIndices.data(), index_count, tangents.data()) == SN_OK);
	for (uint32_t c = 0; c < index_count; ++c)
	{
		const float *t = &tangents[c * 4], *n = &cube.mNormals[cube.mIndices[c] * 3];
		CHECK(std::abs(t[0] * n[0] + t[1] * n[1] + t[2] * n[2]) < 1e-4f); // Tangent lies in the face
	}

	std::vector<uint32_t> indices = cube.mIndices;
	REQUIRE(st_meshopt_optimize(indices.data(), index_count, cube.mPositions.data(), vertex_count, 12) == SN_OK);
	std::vector<uint8_t> encoded(st_meshopt_encode_index_bound(index_count, vertex_count));
	CHECK(st_meshopt_encode_indices(encoded.data(), uint32_t(encoded.size()), indices.data(), index_count) > 0);

	int32_t task = st_uv_unwrap(cube.mPositions.data(), cube.mNormals.data(), vertex_count, cube.mIndices.data(), index_count, 32.0f);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	CHECK(sn_task_result(task, 0) > 0);
	CHECK(sn_task_output_size(task, 2) == index_count * 4);
	sn_task_release(task);

	task = st_convex_decompose(cube.mPositions.data(), vertex_count, cube.mIndices.data(), index_count, 4, 10000, 32);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	CHECK(sn_task_result(task, 0) >= 1);
	sn_task_release(task);
}

TEST_CASE("Basis encodes a KTX2 file")
{
	ModuleScope module;
	constexpr uint32_t size = 64;
	std::vector<uint8_t> rgba;
	for (uint32_t y = 0; y < size; ++y)
		for (uint32_t x = 0; x < size; ++x)
			rgba.insert(rgba.end(), { uint8_t(x * 4), uint8_t(y * 4), 128, 255 });

	int32_t task = st_basis_encode(rgba.data(), size, size, ST_BASIS_UASTC, ST_BASIS_FLAG_MIPMAPS, 50, 2);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	REQUIRE(sn_task_output_size(task, 0) > 12);
	const uint8_t ktx2_magic[] = { 0xAB, 'K', 'T', 'X', ' ', '2', '0', 0xBB };
	CHECK(std::memcmp(sn_task_output_data(task, 0), ktx2_magic, sizeof(ktx2_magic)) == 0);
	// Transcoding the result is covered by the JS tests, which load both modules.
	sn_task_release(task);
}

TEST_CASE("PNG decodes to RGBA8 and garbage fails")
{
	ModuleScope module;
	// 2x2 的 RGBA PNG：红、绿 / 蓝、半透明白
	const uint8_t png[] = {
		0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d, 0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x02,
		0x00, 0x00, 0x00, 0x02, 0x08, 0x06, 0x00, 0x00, 0x00, 0x72, 0xb6, 0x0d, 0x24, 0x00, 0x00, 0x00, 0x14, 0x49, 0x44, 0x41,
		0x54, 0x78, 0x9c, 0x63, 0xf8, 0xcf, 0xc0, 0xf0, 0x1f, 0x0c, 0x19, 0x18, 0xfe, 0x83, 0x40, 0x03, 0x00, 0x49, 0x49, 0x09,
		0x78, 0x9c, 0x3d, 0x3c, 0xbe, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44, 0xae, 0x42, 0x60, 0x82 };
	int32_t task = st_image_decode(png, sizeof(png));
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	CHECK(sn_task_result(task, 0) == 2);
	CHECK(sn_task_result(task, 1) == 2);
	CHECK(sn_task_result(task, 2) == 4);
	REQUIRE(sn_task_output_size(task, 0) == 16);
	const uint8_t expected[] = { 255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 128 };
	CHECK(std::memcmp(sn_task_output_data(task, 0), expected, 16) == 0);
	sn_task_release(task);

	const uint8_t garbage[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	task = st_image_decode(garbage, sizeof(garbage));
	CHECK(WaitForTask(task) == SN_TASK_FAILED);
	sn_task_release(task);
}

TEST_CASE("Cubemap prefiltering keeps a constant sky constant and SH evaluates back to it")
{
	ModuleScope module;
	constexpr uint32_t size = 16;
	// 恒定的天空 (0.5, 1, 2)；+Y 面（第 2 个面）再亮一些，检查方向
	std::vector<float> faces(6 * size * size * 4);
	for (size_t i = 0; i < faces.size(); i += 4)
	{
		bool up = i / (size * size * 4) == 2;
		faces[i] = 0.5f;
		faces[i + 1] = 1.0f;
		faces[i + 2] = up ? 6.0f : 2.0f;
		faces[i + 3] = 1.0f;
	}

	int32_t task = st_cubemap_prefilter_ggx(faces.data(), size, 32, 64);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	// 32、16、8、4、2、1 共 6 级
	uint32_t floats = 0;
	for (uint32_t edge = 32; edge >= 1; edge >>= 1)
		floats += 6 * edge * edge * 4;
	REQUIRE(sn_task_output_size(task, 0) == floats * 4);
	const float *levels = static_cast<const float *>(sn_task_output_data(task, 0));
	// 红、绿通道处处恒定，滤波后不变；蓝通道在最粗糙的一级里向上的面仍然更亮
	for (uint32_t i = 0; i < floats; i += 4)
	{
		CHECK(levels[i] == doctest::Approx(0.5f).epsilon(0.01));
		CHECK(levels[i + 1] == doctest::Approx(1.0f).epsilon(0.01));
	}
	const float *last = levels + floats - 6 * 4;
	CHECK(last[2 * 4 + 2] > last[3 * 4 + 2] + 1.0f);
	sn_task_release(task);

	task = st_cubemap_sh_lambert(faces.data(), size);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	REQUIRE(sn_task_output_size(task, 0) == 27 * 4);
	const float *sh = static_cast<const float *>(sn_task_output_data(task, 0));
	CHECK(sh[0] == doctest::Approx(0.5f).epsilon(0.01));
	CHECK(sh[1] == doctest::Approx(1.0f).epsilon(0.01));
	CHECK(std::abs(sh[3 * 1 + 0]) < 0.01f);
	// 系数 1 乘的是 y：亮的 +Y 面让它在蓝通道是正的
	CHECK(sh[3 * 1 + 2] > 0.5f);
	sn_task_release(task);
}

TEST_CASE("MSDF glyphs are generated in parallel")
{
	const char *font_path = std::getenv("STRIDE_TEST_FONT") != nullptr ? std::getenv("STRIDE_TEST_FONT") : "C:/Windows/Fonts/arial.ttf";
	std::vector<char> font = ReadFile(font_path);
	if (font.empty())
	{
		MESSAGE("skipped: no font at " << font_path);
		return;
	}

	ModuleScope module;
	const uint32_t codepoints[] = { 'A', 'g', ' ', '@' };
	int32_t task = st_font_msdf(font.data(), uint32_t(font.size()), codepoints, 4, 32.0f, 4.0f);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	REQUIRE(sn_task_output_count(task) == 5);
	const st_glyph_metrics *metrics = static_cast<const st_glyph_metrics *>(sn_task_output_data(task, 0));
	CHECK(metrics[0].width > 10);
	CHECK(sn_task_output_size(task, 1) == metrics[0].width * metrics[0].height * 3);
	CHECK(metrics[2].width == 0);	// Space has no outline
	CHECK(metrics[2].advance > 0);
	sn_task_release(task);
}

TEST_CASE("ozz skeleton and animation archives build")
{
	ModuleScope module;
	st_joint joints[2] = {
		{ -1, { 0, 0, 0 }, { 0, 0, 0, 1 }, { 1, 1, 1 } },
		{ 0, { 0, 1, 0 }, { 0, 0, 0, 1 }, { 1, 1, 1 } },
	};
	void *skeleton = nullptr;
	uint32_t skeleton_size = 0;
	REQUIRE(st_anim_build_skeleton(joints, 2, nullptr, &skeleton, &skeleton_size) == SN_OK);
	CHECK(skeleton_size > 0);
	sn_free(skeleton);

	// Joint 0 rotates 90 degrees around Z over one second; joint 1 keeps its rest pose.
	const uint32_t key_counts[] = { 1, 2, 1, 1, 1, 1 };
	const float s = 0.70710678f;
	const float keys[] = {
		0, 0, 0, 0,				0, 0, 0, 0, 1,		1, 0, 0, s, s,		0, 1, 1, 1,
		0, 0, 1, 0,				0, 0, 0, 0, 1,		0, 1, 1, 1,
	};
	void *animation = nullptr;
	uint32_t animation_size = 0;
	REQUIRE(st_anim_build_animation(1.0f, 2, key_counts, keys, &animation, &animation_size) == SN_OK);
	CHECK(animation_size > 0);
	sn_free(animation);
}

TEST_CASE("Navmesh bake produces data")
{
	ModuleScope module;
	const float vertices[] = { -20, 0, -20, 20, 0, -20, 20, 0, 20, -20, 0, 20 };
	const int32_t triangles[] = { 0, 2, 1, 0, 3, 2 };
	sn_nav_config config;
	sn_nav_default_config(&config);
	int32_t task = st_nav_bake(vertices, 4, triangles, 2, &config);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	CHECK(sn_task_output_size(task, 0) > 100);
	sn_task_release(task);
}
