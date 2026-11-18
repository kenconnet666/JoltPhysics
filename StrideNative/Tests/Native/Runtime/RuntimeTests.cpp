#include <TestSupport.h>
#include <StrideNative/Runtime.h>
#include <StrideNative/Values.h>

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

TEST_CASE("Values: dates, date-times and decimals match the stride-ts layouts")
{
	// 和 stride-ts 的 values.test.ts 用同一组对照值（由 JS 的 Date 独立算出）
	struct Case { int32_t year, month, day; sn_date days; };
	const Case cases[] = {
		{ 1970, 1, 1, 0 }, { 1969, 12, 31, -1 }, { 2000, 3, 1, 11017 }, { 2024, 1, 31, 19753 }, { 2024, 2, 29, 19782 },
		{ 1900, 3, 1, -25508 }, { 1600, 2, 29, -135081 }, { -1, 12, 31, -719529 }, { 9999, 12, 31, 2932896 },
	};
	for (const Case &c : cases)
	{
		CHECK(sn_date_from_civil(c.year, c.month, c.day) == c.days);
		int32_t year = 0, month = 0, day = 0;
		sn_date_to_civil(c.days, &year, &month, &day);
		CHECK(year == c.year);
		CHECK(month == c.month);
		CHECK(day == c.day);
	}

	// 1969-12-31T23:00 是负的纳秒数：日期部分要向下取整成 -1，时刻部分是 23:00
	const sn_datetime before_epoch = sn_datetime_from(-1, INT64_C(23) * 3600 * SN_NANOS_PER_SECOND);
	CHECK(before_epoch == -SN_NANOS_PER_SECOND * 3600);
	CHECK(sn_datetime_date(before_epoch) == -1);
	CHECK(sn_datetime_time(before_epoch) == INT64_C(23) * 3600 * SN_NANOS_PER_SECOND);

	CHECK(sn_decimal64_rescale(1250, 2, 4) == 125000);			// 12.50 → 12.5000
	CHECK(sn_decimal64_rescale(1225, 2, 1) == 122);				// 12.25 → 12.2（银行家舍入）
	CHECK(sn_decimal64_rescale(1235, 2, 1) == 124);				// 12.35 → 12.4
	CHECK(sn_decimal64_rescale(-1235, 2, 1) == -124);
	CHECK(sn_decimal64_rescale(1236, 2, 1) == 124);
}

TEST_CASE("Init reports the worker count")
{
	ModuleScope module;
	CHECK(sn_worker_thread_count() >= 0);
	CHECK(sn_init(-1) == SN_ERROR_FAILED);
}

namespace
{
	const sn_box cFloor = { { 0, -1, 0 }, { 50, 1, 50 }, 0 };

	sn_box DynamicBox(float inX, float inY, float inZ)
	{
		return { { inX, inY, inZ }, { 0.5f, 0.5f, 0.5f }, SN_BODY_DYNAMIC };
	}
}

TEST_CASE("Physics box falls onto the floor")
{
	ModuleScope module;
	int32_t world = sn_physics_create(16, 0);
	REQUIRE(world > 0);
	const float *positions = sn_physics_positions(world);
	REQUIRE(positions != nullptr);

	const sn_box boxes[] = { cFloor, DynamicBox(0, 5, 0) };
	REQUIRE(sn_physics_add_boxes(world, boxes, 2) == 0);
	CHECK(positions[4] == doctest::Approx(5.0f));		// written when added, before any step

	for (int i = 0; i < 180; ++i)
		REQUIRE(sn_physics_step(world, 1.0f / 60.0f, 1) == SN_OK);

	CHECK(sn_physics_positions(world) == positions);	// same buffer, rewritten in place
	CHECK(positions[4] == doctest::Approx(0.5f).epsilon(0.05));
	sn_physics_destroy(world);
}

TEST_CASE("Physics world sized to its body count holds a settled stack")
{
	ModuleScope module;
	constexpr int cColumns = 10, cRows = 10, cBodies = cColumns * cRows + 1;
	int32_t world = sn_physics_create(cBodies, 0);
	REQUIRE(world > 0);
	REQUIRE(sn_physics_add_boxes(world, &cFloor, 1) == 0);
	// Columns 1 m apart with 1 m boxes: every box rests on the one below and touches its neighbours.
	std::vector<sn_box> boxes;
	for (int i = 0; i < cColumns * cRows; ++i)
		boxes.push_back(DynamicBox(float(i % cColumns), 0.5f + float(i / cColumns), 0));
	REQUIRE(sn_physics_add_boxes(world, boxes.data(), uint32_t(boxes.size())) == 1);

	for (int i = 0; i < 120; ++i)
		REQUIRE(sn_physics_step(world, 1.0f / 60.0f, 1) == SN_OK);
	sn_physics_destroy(world);
}

TEST_CASE("Physics batch over capacity adds nothing")
{
	ModuleScope module;
	int32_t world = sn_physics_create(4, 0);
	REQUIRE(world > 0);
	std::vector<sn_box> boxes(5, DynamicBox(0, 1, 0));
	CHECK(sn_physics_add_boxes(world, boxes.data(), 5) == SN_ERROR_FAILED);
	CHECK(sn_physics_add_boxes(world, boxes.data(), 4) == 0);
	CHECK(sn_physics_add_boxes(world, boxes.data(), 1) == SN_ERROR_FAILED);
	sn_physics_destroy(world);
}

namespace
{
	const float cHalfSqrt2 = std::sqrt(0.5f);

	/// 绕 Y 轴转 90°：x 轴转到 -z。
	void SetQuarterTurnAroundY(float *outRotation)
	{
		outRotation[0] = 0;
		outRotation[1] = cHalfSqrt2;
		outRotation[2] = 0;
		outRotation[3] = cHalfSqrt2;
	}

	void CheckTranslation(const float *inWorld, float inX, float inY, float inZ, double inEpsilon = 1e-4)
	{
		CHECK(inWorld[12] == doctest::Approx(inX).epsilon(inEpsilon));
		CHECK(inWorld[13] == doctest::Approx(inY).epsilon(inEpsilon));
		CHECK(inWorld[14] == doctest::Approx(inZ).epsilon(inEpsilon));
	}
}

TEST_CASE("变换：世界矩阵按层级传播，挂接和释放都做检查")
{
	ModuleScope module;
	int32_t store = sn_transforms_create(4);
	REQUIRE(store > 0);
	sn_transform_columns c {};
	REQUIRE(sn_transforms_columns(store, &c) == SN_OK);
	CHECK(c.capacity == 4);

	int32_t slots[3] = {};
	REQUIRE(sn_transforms_alloc(store, slots, 3) == 3);
	CHECK((slots[0] == 1 && slots[1] == 2 && slots[2] == 3));
	// 槽位号最小的是孙节点：计算顺序要按层级，不能按槽位号
	const int32_t grandchild = slots[0], child = slots[1], root = slots[2];
	REQUIRE(sn_transforms_set_parent(store, grandchild, child) == SN_OK);
	REQUIRE(sn_transforms_set_parent(store, child, root) == SN_OK);
	CHECK(c.parent[child] == root);
	CHECK(sn_transforms_slot_limit(store) == 3);

	// 根：平移 (10, 0, 0)，绕 Y 转 90°，缩放 2；子：(1, 0, 0)；孙：(0, 1, 0)
	c.position[root * 3] = 10;
	SetQuarterTurnAroundY(&c.rotation[root * 4]);
	std::fill_n(&c.scale[root * 3], 3, 2.0f);
	c.position[child * 3] = 1;
	c.position[grandchild * 3 + 1] = 1;
	REQUIRE(sn_transforms_update(store) == SN_OK);
	// 子节点的 (1, 0, 0) 缩放成 (2, 0, 0)，转成 (0, 0, -2)，再平移
	CheckTranslation(&c.world[child * 16], 10, 0, -2);
	// 孙节点的 (0, 1, 0) 经过根的缩放成 (0, 2, 0)
	CheckTranslation(&c.world[grandchild * 16], 10, 2, -2);

	CHECK(sn_transforms_set_parent(store, root, grandchild) == SN_ERROR_INVALID_ARGUMENT);	// 成环
	CHECK(sn_transforms_set_parent(store, child, child) == SN_ERROR_INVALID_ARGUMENT);
	CHECK(sn_transforms_set_parent(store, child, 4) == SN_ERROR_INVALID_ARGUMENT);		// 没分配
	CHECK(sn_transforms_free(store, &root, 1) == SN_ERROR_INVALID_ARGUMENT);				// 还有子节点
	const int32_t twice[] = { grandchild, grandchild };
	CHECK(sn_transforms_free(store, twice, 2) == SN_ERROR_INVALID_ARGUMENT);
	CHECK(c.parent[child] == root);														// 失败的释放什么都没改

	int32_t last = 0;
	REQUIRE(sn_transforms_alloc(store, &last, 1) == 1);
	CHECK(last == 4);
	CHECK(sn_transforms_alloc(store, &last, 1) == SN_ERROR_FAILED);						// 容量用完

	// 父节点和子节点一起释放可以，顺序不限
	const int32_t subtree[] = { root, grandchild, child };
	REQUIRE(sn_transforms_free(store, subtree, 3) == SN_OK);
	CHECK(sn_transforms_free(store, &root, 1) == SN_ERROR_INVALID_ARGUMENT);
	int32_t reused = 0;
	REQUIRE(sn_transforms_alloc(store, &reused, 1) == 1);
	CHECK(reused == child);																// 后进先出地重用
	CHECK(c.position[reused * 3] == 0.0f);												// 重新分配时恢复默认值
	CHECK(c.parent[reused] == 0);
	REQUIRE(sn_transforms_update(store) == SN_OK);
	CheckTranslation(&c.world[reused * 16], 0, 0, 0);
	sn_transforms_destroy(store);
}

TEST_CASE("物理：绑定变换的物体从槽位出发，步进后把位姿写回槽位")
{
	ModuleScope module;
	int32_t store = sn_transforms_create(8);
	REQUIRE(store > 0);
	sn_transform_columns c {};
	REQUIRE(sn_transforms_columns(store, &c) == SN_OK);
	int32_t slots[3] = {};
	REQUIRE(sn_transforms_alloc(store, slots, 3) == 3);
	const int32_t root = slots[0], parent = slots[1], child = slots[2];

	// 根上的箱子从 (0, 5, 0) 落下；另一个挂在平移 (10, 0, 0)、绕 Y 转 90° 的父节点下，
	// 本地 (0, 4, 2) 转成 (2, 4, 0)，世界位置 (12, 4, 0)
	c.position[root * 3 + 1] = 5;
	c.position[parent * 3] = 10;
	SetQuarterTurnAroundY(&c.rotation[parent * 4]);
	REQUIRE(sn_transforms_set_parent(store, child, parent) == SN_OK);
	c.position[child * 3 + 1] = 4;
	c.position[child * 3 + 2] = 2;

	int32_t world = sn_physics_create(8, store);
	REQUIRE(world > 0);
	sn_box root_box = DynamicBox(0, 0, 0), child_box = DynamicBox(0, 0, 0);
	root_box.transform = root;
	child_box.transform = child;
	const sn_box boxes[] = { cFloor, root_box, child_box };
	REQUIRE(sn_physics_add_boxes(world, boxes, 3) == 0);
	const float *positions = sn_physics_positions(world);
	CHECK(positions[1 * 3 + 1] == doctest::Approx(5.0f));
	CHECK(positions[2 * 3 + 0] == doctest::Approx(12.0f));
	CHECK(positions[2 * 3 + 1] == doctest::Approx(4.0f));

	// 帧的顺序：物理步进写回本地位姿，然后更新世界矩阵
	for (int i = 0; i < 180; ++i)
	{
		REQUIRE(sn_physics_step(world, 1.0f / 60.0f, 1) == SN_OK);
		REQUIRE(sn_transforms_update(store) == SN_OK);
	}
	CHECK(c.position[root * 3 + 1] == doctest::Approx(0.5f).epsilon(0.05));
	CheckTranslation(&c.world[root * 16], 0, 0.5f, 0, 0.05);
	// 子节点写回的是父节点空间里的位姿：世界里落在 (12, 0.5, 0)；它一开始就带着父节点的 90°，
	// 转过的箱子照样平着落地，所以本地旋转仍是单位四元数，世界旋转仍是父节点的那 90°
	CheckTranslation(&c.world[child * 16], 12, 0.5f, 0, 0.05);
	CHECK(c.position[child * 3 + 1] == doctest::Approx(0.5f).epsilon(0.05));
	CHECK(std::abs(c.rotation[child * 4 + 3]) == doctest::Approx(1.0f).epsilon(1e-3));
	CHECK(c.world[child * 16 + 2] == doctest::Approx(-1.0f).epsilon(1e-3));		// x 轴转到 -z
	CHECK(c.world[child * 16 + 8] == doctest::Approx(1.0f).epsilon(1e-3));		// z 轴转到 x

	sn_box unallocated = DynamicBox(0, 1, 0);
	unallocated.transform = 7;
	CHECK(sn_physics_add_boxes(world, &unallocated, 1) == SN_ERROR_INVALID_ARGUMENT);
	int32_t plain = sn_physics_create(2, 0);
	CHECK(sn_physics_add_boxes(plain, &root_box, 1) == SN_ERROR_INVALID_ARGUMENT);	// 世界没有变换存储
	CHECK(sn_physics_create(2, 99) == SN_ERROR_INVALID_HANDLE);
	sn_physics_destroy(plain);

	sn_transforms_destroy(store);
	CHECK(sn_physics_step(world, 1.0f / 60.0f, 1) == SN_ERROR_INVALID_HANDLE);			// 存储比世界先销毁
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
	const float *positions = sn_crowd_positions(crowd);
	REQUIRE(positions != nullptr);
	CHECK(std::isnan(positions[0]));
	const float agent_start[] = { -20, 0, 0 }, target[] = { 20, 0, 0 };
	int32_t agent = sn_crowd_add_agent(crowd, agent_start, 0.5f, 2.0f, 3.5f, 8.0f);
	REQUIRE(agent >= 0);
	CHECK(positions[agent * 3] == doctest::Approx(-20.0f).epsilon(0.05));
	REQUIRE(sn_crowd_request_move(crowd, agent, target) == SN_OK);
	for (int i = 0; i < 450; ++i) // 15 s at 3.5 m/s covers the 40 m to the target
		sn_crowd_update(crowd, 1.0f / 30.0f);
	CHECK(sn_crowd_positions(crowd) == positions);
	CHECK(positions[agent * 3] == doctest::Approx(20.0f).epsilon(0.05));
	sn_crowd_remove_agent(crowd, agent);
	CHECK(std::isnan(positions[agent * 3]));

	sn_crowd_destroy(crowd);
	sn_nav_destroy(loaded);
	sn_nav_destroy(navmesh);
}
