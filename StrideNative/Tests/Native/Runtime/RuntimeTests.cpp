#include <TestSupport.h>
#include <StrideNative/Runtime.h>

#include <meshoptimizer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

namespace
{
	std::vector<uint32_t> NormalizeTriangles(std::vector<uint32_t> inIndices)
	{
		for (size_t i = 0; i + 2 < inIndices.size(); i += 3)
			while (inIndices[i] > inIndices[i + 1] || inIndices[i] > inIndices[i + 2])
				std::rotate(inIndices.begin() + i, inIndices.begin() + i + 1, inIndices.begin() + i + 3);
		return inIndices;
	}

	/// A flat square of size inSize centered on the origin, split into a grid of quads.
	void MakeGround(float inSize, int inCells, std::vector<float> &outVertices, std::vector<int32_t> &outTriangles)
	{
		for (int z = 0; z <= inCells; ++z)
			for (int x = 0; x <= inCells; ++x)
				outVertices.insert(outVertices.end(), { -inSize / 2 + inSize * x / inCells, 0.0f, -inSize / 2 + inSize * z / inCells });
		for (int z = 0; z < inCells; ++z)
			for (int x = 0; x < inCells; ++x)
			{
				int32_t i = z * (inCells + 1) + x;
				outTriangles.insert(outTriangles.end(), { i, i + inCells + 1, i + 1, i + 1, i + inCells + 1, i + inCells + 2 });
			}
	}
}

TEST_CASE("Init reports the worker count")
{
	ModuleScope module;
	CHECK(sn_worker_thread_count() >= 0);
	CHECK(sn_init(-1) == SN_ERROR_FAILED);
}

TEST_CASE("Physics box falls onto the floor")
{
	ModuleScope module;
	int32_t world = sn_physics_create(16);
	REQUIRE(world > 0);
	const float floor_position[] = { 0, -1, 0 }, floor_half[] = { 50, 1, 50 };
	const float box_position[] = { 0, 5, 0 }, box_half[] = { 0.5f, 0.5f, 0.5f };
	REQUIRE(sn_physics_add_box(world, floor_position, floor_half, 0) == 0);
	REQUIRE(sn_physics_add_box(world, box_position, box_half, 1) == 1);

	for (int i = 0; i < 180; ++i)
		REQUIRE(sn_physics_step(world, 1.0f / 60.0f, 1) == SN_OK);

	float positions[6];
	REQUIRE(sn_physics_get_positions(world, positions, 2) == 2);
	CHECK(positions[4] == doctest::Approx(0.5f).epsilon(0.05));
	sn_physics_destroy(world);
}

TEST_CASE("Meshopt batch decodes vertex and index buffers in parallel")
{
	ModuleScope module;
	constexpr uint32_t vertex_count = 1000;
	std::vector<float> vertices(vertex_count * 4);
	for (uint32_t i = 0; i < vertices.size(); ++i)
		vertices[i] = std::sin(float(i));
	std::vector<uint32_t> indices;
	for (uint32_t i = 0; i + 2 < vertex_count; ++i)
		indices.insert(indices.end(), { i, i + 1, i + 2 });

	std::vector<unsigned char> encoded_vertices(meshopt_encodeVertexBufferBound(vertex_count, 16));
	encoded_vertices.resize(meshopt_encodeVertexBuffer(encoded_vertices.data(), encoded_vertices.size(), vertices.data(), vertex_count, 16));
	std::vector<unsigned char> encoded_indices(meshopt_encodeIndexBufferBound(indices.size(), vertex_count));
	encoded_indices.resize(meshopt_encodeIndexBuffer(encoded_indices.data(), encoded_indices.size(), indices.data(), indices.size()));

	std::vector<float> decoded_vertices(vertices.size());
	std::vector<uint32_t> decoded_indices(indices.size());
	sn_meshopt_request requests[2] = {
		{ decoded_vertices.data(), vertex_count, 16, encoded_vertices.data(), uint32_t(encoded_vertices.size()), SN_MESHOPT_MODE_ATTRIBUTES, SN_MESHOPT_FILTER_NONE, 1 },
		{ decoded_indices.data(), uint32_t(indices.size()), 4, encoded_indices.data(), uint32_t(encoded_indices.size()), SN_MESHOPT_MODE_TRIANGLES, SN_MESHOPT_FILTER_NONE, 1 },
	};
	int32_t task = sn_meshopt_decode_batch(requests, 2);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	CHECK(requests[0].result == SN_OK);
	CHECK(requests[1].result == SN_OK);
	CHECK(std::memcmp(vertices.data(), decoded_vertices.data(), vertices.size() * 4) == 0);
	// The index codec may rotate a triangle's corners (keeping its winding), so compare rotated forms.
	CHECK(NormalizeTriangles(indices) == NormalizeTriangles(decoded_indices));
	sn_task_release(task);
}

TEST_CASE("Navmesh builds in tiles, finds paths, serializes and drives a crowd")
{
	ModuleScope module;
	std::vector<float> vertices;
	std::vector<int32_t> triangles;
	MakeGround(60.0f, 30, vertices, triangles);

	sn_nav_config config;
	sn_nav_default_config(&config);
	config.tile_size = 32;
	int32_t task = sn_nav_build(vertices.data(), uint32_t(vertices.size() / 3), triangles.data(), uint32_t(triangles.size() / 3), &config);
	REQUIRE(WaitForTask(task) == SN_TASK_DONE);
	int32_t navmesh = sn_task_result(task, 0);
	CHECK(sn_task_result(task, 1) > 4);
	sn_task_release(task);

	const float start[] = { -25, 0, -25 }, end[] = { 25, 0, 25 }, extents[] = { 2, 4, 2 };
	float points[3 * 32];
	int32_t point_count = sn_nav_find_path(navmesh, start, end, extents, points, 32);
	REQUIRE(point_count >= 2);
	CHECK(points[(point_count - 1) * 3] == doctest::Approx(25.0f).epsilon(0.05));

	void *data = nullptr;
	uint32_t size = 0;
	REQUIRE(sn_nav_serialize(navmesh, &data, &size) == SN_OK);
	int32_t loaded = sn_nav_load(data, size);
	sn_free(data);
	REQUIRE(loaded > 0);
	CHECK(sn_nav_find_path(loaded, start, end, extents, points, 32) == point_count);

	int32_t crowd = sn_crowd_create(loaded, 8, 0.6f);
	REQUIRE(crowd > 0);
	const float agent_start[] = { -20, 0, 0 }, target[] = { 20, 0, 0 };
	int32_t agent = sn_crowd_add_agent(crowd, agent_start, 0.5f, 2.0f, 3.5f, 8.0f);
	REQUIRE(agent >= 0);
	REQUIRE(sn_crowd_request_move(crowd, agent, target) == SN_OK);
	for (int i = 0; i < 450; ++i) // 15 s at 3.5 m/s covers the 40 m to the target
		sn_crowd_update(crowd, 1.0f / 30.0f);
	float agent_position[3];
	REQUIRE(sn_crowd_get_positions(crowd, agent_position, 1) == 1);
	CHECK(agent_position[0] == doctest::Approx(20.0f).epsilon(0.05));

	sn_crowd_destroy(crowd);
	sn_nav_destroy(loaded);
	sn_nav_destroy(navmesh);
}
