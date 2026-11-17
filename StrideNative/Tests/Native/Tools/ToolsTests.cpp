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
