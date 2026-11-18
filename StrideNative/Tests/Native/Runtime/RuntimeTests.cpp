#include <TestSupport.h>
#include <StrideNative/Debug.h>
#include <StrideNative/Navigation.h>
#include <StrideNative/Physics.h>
#include <StrideNative/Runtime.h>
#include <StrideNative/Texture.h>
#include <StrideNative/Values.h>

#include <meshoptimizer.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>
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
	/// 一个盒子一个物体（测试用的简写），AddBoxes 把它们换成 sn_body 加 sn_shape
	struct TestBox
	{
		float position[3];
		float half_extent[3];
		uint32_t motion;
		int32_t transform = 0;
	};

	const TestBox cFloor = { { 0, -1, 0 }, { 50, 1, 50 }, SN_MOTION_STATIC };

	TestBox DynamicBox(float inX, float inY, float inZ)
	{
		return { { inX, inY, inZ }, { 0.5f, 0.5f, 0.5f }, SN_MOTION_DYNAMIC };
	}

	sn_shape BoxShape(float inX, float inY, float inZ, float inMass = 1.0f)
	{
		sn_shape shape {};
		shape.type = SN_SHAPE_BOX;
		shape.rotation[3] = 1.0f;
		shape.size[0] = inX;
		shape.size[1] = inY;
		shape.size[2] = inZ;
		shape.mass = inMass;
		return shape;
	}

	sn_body Body(uint32_t inMotion, float inX, float inY, float inZ, uint32_t inFirstShape, uint32_t inShapeCount = 1, uint32_t inLayer = 0)
	{
		sn_body body {};
		body.motion = inMotion;
		body.position[0] = inX;
		body.position[1] = inY;
		body.position[2] = inZ;
		body.rotation[3] = 1.0f;
		body.first_shape = inFirstShape;
		body.shape_count = inShapeCount;
		body.layer = inLayer;
		body.friction = 0.5f;
		return body;
	}

	int32_t AddBoxes(int32_t inWorld, const TestBox *inBoxes, uint32_t inCount)
	{
		std::vector<sn_body> bodies;
		std::vector<sn_shape> shapes;
		for (uint32_t i = 0; i < inCount; ++i)
		{
			const TestBox &box = inBoxes[i];
			shapes.push_back(BoxShape(box.half_extent[0] * 2, box.half_extent[1] * 2, box.half_extent[2] * 2));
			bodies.push_back(Body(box.motion, box.position[0], box.position[1], box.position[2], i));
			bodies.back().transform = box.transform;
		}
		return sn_physics_add_bodies(inWorld, bodies.data(), inCount, shapes.data(), inCount, nullptr, 0, nullptr);
	}
}

TEST_CASE("Physics box falls onto the floor")
{
	ModuleScope module;
	int32_t world = sn_physics_create(16, 0, nullptr);
	REQUIRE(world > 0);
	const float *positions = sn_physics_positions(world);
	REQUIRE(positions != nullptr);

	const TestBox boxes[] = { cFloor, DynamicBox(0, 5, 0) };
	REQUIRE(AddBoxes(world, boxes, 2) == 0);
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
	int32_t world = sn_physics_create(cBodies, 0, nullptr);
	REQUIRE(world > 0);
	REQUIRE(AddBoxes(world, &cFloor, 1) == 0);
	// Columns 1 m apart with 1 m boxes: every box rests on the one below and touches its neighbours.
	std::vector<TestBox> boxes;
	for (int i = 0; i < cColumns * cRows; ++i)
		boxes.push_back(DynamicBox(float(i % cColumns), 0.5f + float(i / cColumns), 0));
	REQUIRE(AddBoxes(world, boxes.data(), uint32_t(boxes.size())) == 1);

	for (int i = 0; i < 120; ++i)
		REQUIRE(sn_physics_step(world, 1.0f / 60.0f, 1) == SN_OK);
	sn_physics_destroy(world);
}

TEST_CASE("Physics batch over capacity adds nothing")
{
	ModuleScope module;
	int32_t world = sn_physics_create(4, 0, nullptr);
	REQUIRE(world > 0);
	std::vector<TestBox> boxes(5, DynamicBox(0, 1, 0));
	CHECK(AddBoxes(world, boxes.data(), 5) == SN_ERROR_FAILED);
	CHECK(AddBoxes(world, boxes.data(), 4) == 0);
	CHECK(AddBoxes(world, boxes.data(), 1) == SN_ERROR_FAILED);
	sn_physics_destroy(world);
}

namespace
{
	/// Third Person Platformer 的设置：重力 -40、阻尼 0.05、全部层相互碰撞
	sn_physics_settings PlatformerSettings()
	{
		sn_physics_settings settings {};
		settings.gravity[1] = -40.0f;
		settings.linear_damping = 0.05f;
		settings.angular_damping = 0.05f;
		std::fill(std::begin(settings.collision_masks), std::end(settings.collision_masks), 0xffffffffu);
		return settings;
	}

	/// TPP 的角色：胶囊半径 0.4、长 1.2，中心在脚底上方 0.95
	struct TestCharacter
	{
		sn_shape shape {};
		sn_character settings { 200.0f, 1000.0f, 13.0f, 50.0f, 0.2f, 0.2f };
		sn_body body {};

		TestCharacter(float inX, float inY, float inZ)
		{
			shape.type = SN_SHAPE_CAPSULE;
			shape.position[1] = 0.95f;
			shape.rotation[3] = 1.0f;
			shape.size[0] = 0.4f;
			shape.size[1] = 1.2f;
			shape.mass = 1.0f;
			body = Body(SN_MOTION_CHARACTER, inX, inY, inZ, 0, 1, 2);
		}

		int32_t Add(int32_t inWorld) const
		{
			return sn_physics_add_bodies(inWorld, &body, 1, &shape, 1, &settings, 1, nullptr);
		}
	};

	void Step(int32_t inWorld, int inSteps)
	{
		for (int i = 0; i < inSteps; ++i)
			REQUIRE(sn_physics_step(inWorld, 1.0f / 60.0f, 1) == SN_OK);
	}
}

TEST_CASE("物理：角色站稳、按目标速度走、起跳后落回地面（Stride 的 CharacterComponent）")
{
	ModuleScope module;
	sn_physics_settings settings = PlatformerSettings();
	int32_t world = sn_physics_create(8, 0, &settings);
	REQUIRE(world > 0);
	REQUIRE(AddBoxes(world, &cFloor, 1) == 0);
	TestCharacter player(0, 0.05f, 0);
	int32_t character = player.Add(world);
	REQUIRE(character == 1);
	const float *positions = sn_physics_positions(world);

	Step(world, 30);
	CHECK(sn_physics_character_grounded(world, character) == 1);
	CHECK(positions[4] == doctest::Approx(0.0f).epsilon(0.05));

	// 目标速度 10 m/s：最大水平力 200、质量 1，0.05 秒就能到，一秒走将近 10 米
	REQUIRE(sn_physics_character_move(world, character, 10.0f, 0.0f, 0) == SN_OK);
	Step(world, 60);
	CHECK(positions[3] > 9.0f);
	CHECK(positions[3] < 10.5f);
	CHECK(std::abs(positions[5]) < 0.01f);
	CHECK(sn_physics_character_grounded(world, character) == 1);

	// 起跳：竖直速度补到 13，重力 -40 下大约升高 13² / 80 ≈ 2.1 米
	REQUIRE(sn_physics_character_move(world, character, 0.0f, 0.0f, 1) == SN_OK);
	Step(world, 2);
	CHECK(sn_physics_character_grounded(world, character) == 0);
	float highest = 0.0f;
	for (int i = 0; i < 30; ++i)
	{
		Step(world, 1);
		highest = std::max(highest, positions[4]);
	}
	CHECK(highest == doctest::Approx(2.1f).epsilon(0.15));
	Step(world, 60);
	CHECK(sn_physics_character_grounded(world, character) == 1);
	CHECK(positions[4] == doctest::Approx(0.0f).epsilon(0.05));

	// 空中操控：原地起跳，到空中再推方向，最多加速到目标速度的 0.2（2 m/s），加速度是 200 · 0.2 / 1 = 40
	const float before_x = positions[3];
	REQUIRE(sn_physics_character_move(world, character, 0.0f, 0.0f, 1) == SN_OK);
	Step(world, 3);
	REQUIRE(sn_physics_character_grounded(world, character) == 0);
	REQUIRE(sn_physics_character_move(world, character, 10.0f, 0.0f, 0) == SN_OK);
	Step(world, 15);
	CHECK(positions[3] - before_x > 0.3f);
	CHECK(positions[3] - before_x < 0.6f);
	REQUIRE(sn_physics_character_move(world, character, 0.0f, 0.0f, 0) == SN_OK);
	Step(world, 60);

	// 不是角色的物体、不存在的物体都不能当角色用
	CHECK(sn_physics_character_move(world, 0, 1.0f, 0.0f, 0) == SN_ERROR_INVALID_ARGUMENT);
	CHECK(sn_physics_character_grounded(world, 5) == SN_ERROR_INVALID_ARGUMENT);
	sn_physics_destroy(world);
}

TEST_CASE("物理：碰撞层的矩阵和查询的遮罩，射线和球体扫掠")
{
	ModuleScope module;
	sn_physics_settings settings = PlatformerSettings();
	// 层 0 和层 2 不相互碰撞
	settings.collision_masks[0] &= ~(1u << 2);
	settings.collision_masks[2] &= ~(1u << 0);
	int32_t world = sn_physics_create(8, 0, &settings);
	REQUIRE(world > 0);

	// 层 0 的墙在 x = 5，层 1 的墙在 x = 10，都是 1 米厚
	const sn_shape walls[] = { BoxShape(1, 10, 10), BoxShape(1, 10, 10) };
	const sn_body bodies[] = { Body(SN_MOTION_STATIC, 5, 0, 0, 0, 1, 0), Body(SN_MOTION_STATIC, 10, 0, 0, 1, 1, 1) };
	REQUIRE(sn_physics_add_bodies(world, bodies, 2, walls, 2, nullptr, 0, nullptr) == 0);

	const float origin[3] = { 0, 0, 0 }, along_x[3] = { 2, 0, 0 };
	sn_hit hit {};
	REQUIRE(sn_physics_ray_cast(world, origin, along_x, 100, 1u << 0, &hit) == 1);
	CHECK(hit.distance == doctest::Approx(4.5f));
	CHECK(hit.body == 0);
	CHECK(hit.normal[0] == doctest::Approx(-1.0f));
	REQUIRE(sn_physics_ray_cast(world, origin, along_x, 100, 1u << 1, &hit) == 1);
	CHECK(hit.distance == doctest::Approx(9.5f));
	CHECK(hit.body == 1);
	CHECK(sn_physics_ray_cast(world, origin, along_x, 100, 0, &hit) == 0);
	CHECK(sn_physics_ray_cast(world, origin, along_x, 4, 1u << 0, &hit) == 0);

	// 半径 0.5 的球：碰到 x = 4.5 的墙面时球心在 4
	REQUIRE(sn_physics_sphere_cast(world, 0.5f, origin, along_x, 100, 0xffffffffu, 0, &hit) == 1);
	CHECK(hit.distance == doctest::Approx(4.0f).epsilon(0.01));
	CHECK(hit.body == 0);
	const float inside[3] = { 5, 0, 0 };
	REQUIRE(sn_physics_sphere_cast(world, 0.5f, inside, along_x, 100, 1u << 0, 0, &hit) == 1);
	CHECK(hit.distance == doctest::Approx(0.0f));
	// 忽略起点就重叠的墙：打到 x = 10 的那面（球心停在 9）
	REQUIRE(sn_physics_sphere_cast(world, 0.5f, inside, along_x, 100, 0xffffffffu, SN_CAST_IGNORE_INITIAL_OVERLAP, &hit) == 1);
	CHECK(hit.distance == doctest::Approx(4.0f).epsilon(0.01));
	CHECK(hit.body == 1);
	const float zero[3] = { 0, 0, 0 };
	CHECK(sn_physics_ray_cast(world, origin, zero, 100, 1, &hit) == SN_ERROR_INVALID_ARGUMENT);

	// 层 2 的箱子穿过层 0 的地面掉下去，层 1 的停在地面上
	const sn_shape pieces[] = { BoxShape(100, 2, 100), BoxShape(1, 1, 1), BoxShape(1, 1, 1) };
	const sn_body falling[] = { Body(SN_MOTION_STATIC, 0, -1, 30, 0, 1, 0), Body(SN_MOTION_DYNAMIC, 0, 3, 30, 1, 1, 2), Body(SN_MOTION_DYNAMIC, 3, 3, 30, 2, 1, 1) };
	REQUIRE(sn_physics_add_bodies(world, falling, 3, pieces, 3, nullptr, 0, nullptr) == 2);
	Step(world, 60);
	const float *positions = sn_physics_positions(world);
	CHECK(positions[3 * 3 + 1] < -2.0f);
	CHECK(positions[4 * 3 + 1] == doctest::Approx(0.5f).epsilon(0.05));
	sn_physics_destroy(world);

	// 碰撞矩阵要对称
	settings.collision_masks[3] = 0;
	CHECK(sn_physics_create(8, 0, &settings) == SN_ERROR_INVALID_ARGUMENT);
}

TEST_CASE("物理：网格碰撞体和带本地位姿的复合碰撞体")
{
	ModuleScope module;
	int32_t world = sn_physics_create(8, 0, nullptr);
	REQUIRE(world > 0);

	// 20 x 20 的地面网格（两个三角形，朝上）
	const float vertices[] = { -10, 0, -10, 10, 0, -10, 10, 0, 10, -10, 0, 10 };
	const uint32_t indices[] = { 0, 2, 1, 0, 3, 2 };
	int32_t mesh = sn_physics_create_mesh(world, vertices, 4, indices, 6);
	REQUIRE(mesh > 0);
	CHECK(sn_physics_create_mesh(world, vertices, 4, indices, 5) == SN_ERROR_INVALID_ARGUMENT);
	sn_shape ground {};
	ground.type = SN_SHAPE_MESH;
	ground.rotation[3] = 1.0f;
	ground.mesh = mesh;
	sn_body ground_body = Body(SN_MOTION_STATIC, 0, 0, 0, 0);
	REQUIRE(sn_physics_add_bodies(world, &ground_body, 1, &ground, 1, nullptr, 0, nullptr) == 0);
	TestBox falling = DynamicBox(0, 3, 0);
	REQUIRE(AddBoxes(world, &falling, 1) == 1);
	Step(world, 120);
	CHECK(sn_physics_positions(world)[4] == doctest::Approx(0.5f).epsilon(0.05));

	// 动态物体上的网格只能单独用（复合形状里的网格只能是静态的）
	sn_shape mesh_and_box[] = { ground, BoxShape(1, 1, 1) };
	sn_body dynamic_mesh = Body(SN_MOTION_DYNAMIC, 0, 5, 0, 0, 2);
	CHECK(sn_physics_add_bodies(world, &dynamic_mesh, 1, mesh_and_box, 2, nullptr, 0, nullptr) == SN_ERROR_INVALID_ARGUMENT);

	// 复合：两个盒子在本地 x = ±3，物体绕 Y 转 90°，本地 +x 转到世界 -z
	sn_shape parts[] = { BoxShape(1, 1, 1), BoxShape(1, 1, 1) };
	parts[0].position[0] = -3;
	parts[1].position[0] = 3;
	sn_body compound = Body(SN_MOTION_STATIC, 0, 20, 0, 0, 2);
	compound.rotation[1] = std::sqrt(0.5f);
	compound.rotation[3] = std::sqrt(0.5f);
	REQUIRE(sn_physics_add_bodies(world, &compound, 1, parts, 2, nullptr, 0, nullptr) == 2);
	const float down[3] = { 0, -1, 0 }, above_minus_z[3] = { 0, 25, -3 }, above_plus_x[3] = { 3, 25, 0 };
	sn_hit hit {};
	REQUIRE(sn_physics_ray_cast(world, above_minus_z, down, 10, 0xffffffffu, &hit) == 1);
	CHECK(hit.distance == doctest::Approx(4.5f));
	CHECK(hit.body == 2);
	CHECK(sn_physics_ray_cast(world, above_plus_x, down, 10, 0xffffffffu, &hit) == 0);
	sn_physics_destroy(world);
}

TEST_CASE("物理：凸包和 BodyComponent 的控制（运动学切换、瞬移、速度、冲量、重力、唤醒）")
{
	ModuleScope module;
	int32_t world = sn_physics_create(8, 0, nullptr);
	REQUIRE(world > 0);
	const TestBox floor = cFloor;
	REQUIRE(AddBoxes(world, &floor, 1) == 0);

	// 单位立方体的 8 个角做成凸包，落到地面上，中心停在 0.5
	float corners[24];
	for (int i = 0; i < 8; ++i)
	{
		corners[i * 3] = (i & 1) ? 0.5f : -0.5f;
		corners[i * 3 + 1] = (i & 2) ? 0.5f : -0.5f;
		corners[i * 3 + 2] = (i & 4) ? 0.5f : -0.5f;
	}
	int32_t hull = sn_physics_create_hull(world, corners, 8);
	REQUIRE(hull > 0);
	CHECK(sn_physics_create_hull(world, corners, 3) == SN_ERROR_INVALID_ARGUMENT);
	sn_shape hull_shape {};
	hull_shape.type = SN_SHAPE_CONVEX_HULL;
	hull_shape.rotation[3] = 1.0f;
	hull_shape.mesh = hull;
	hull_shape.mass = 1.0f;
	sn_body crate = Body(SN_MOTION_DYNAMIC, 0, 3, 0, 0);
	crate.flags = SN_BODY_NEVER_SLEEP;
	REQUIRE(sn_physics_add_bodies(world, &crate, 1, &hull_shape, 1, nullptr, 0, nullptr) == 1);
	Step(world, 120);
	const float *positions = sn_physics_positions(world);
	CHECK(positions[4] == doctest::Approx(0.5f).epsilon(0.05));
	CHECK(sn_physics_body_awake(world, 1) == 1);		// 一直醒着

	// 冲量：质量 1，向上 5 → 速度 5
	const float up[3] = { 0, 5, 0 };
	REQUIRE(sn_physics_body_apply_impulse(world, 1, up, nullptr) == SN_OK);
	float linear[3], angular[3];
	REQUIRE(sn_physics_body_velocity(world, 1, linear, angular) == SN_OK);
	CHECK(linear[1] == doctest::Approx(5.0f));

	// 运动学：不受重力和冲量，按速度走；瞬移立即生效
	REQUIRE(sn_physics_body_set_kinematic(world, 1, 1) == SN_OK);
	const float place[3] = { 0, 10, 0 }, identity[4] = { 0, 0, 0, 1 }, zero[3] = { 0, 0, 0 }, sideways[3] = { 1, 0, 0 };
	REQUIRE(sn_physics_body_teleport(world, 1, place, identity) == SN_OK);
	CHECK(positions[4] == 10.0f);
	REQUIRE(sn_physics_body_set_velocity(world, 1, sideways, zero) == SN_OK);
	REQUIRE(sn_physics_body_apply_impulse(world, 1, up, nullptr) == SN_OK);
	Step(world, 60);
	CHECK(positions[3] == doctest::Approx(1.0f).epsilon(0.01));
	CHECK(positions[4] == doctest::Approx(10.0f));

	// 回到动态、关掉重力：停在原地
	REQUIRE(sn_physics_body_set_kinematic(world, 1, 0) == SN_OK);
	REQUIRE(sn_physics_body_set_velocity(world, 1, zero, zero) == SN_OK);
	REQUIRE(sn_physics_body_set_gravity(world, 1, 0) == SN_OK);
	Step(world, 30);
	CHECK(positions[4] == doctest::Approx(10.0f));
	REQUIRE(sn_physics_body_set_gravity(world, 1, 1) == SN_OK);
	Step(world, 30);
	CHECK(positions[4] < 9.0f);

	// 静态物体不能这样控制
	CHECK(sn_physics_body_set_kinematic(world, 0, 1) == SN_ERROR_INVALID_ARGUMENT);
	sn_physics_destroy(world);
}

TEST_CASE("物理：传感器和接触事件（Stride 的 Trigger、IContactHandler），移除物体")
{
	ModuleScope module;
	int32_t world = sn_physics_create(8, 0, nullptr);
	REQUIRE(world > 0);
	// 0：地面（报接触事件），1：地面上方 2..4 的传感器盒子（静态），2：从 8 米落下的盒子
	sn_shape shapes[] = { BoxShape(20, 1, 20), BoxShape(4, 2, 4), BoxShape(1, 1, 1) };
	sn_body bodies[] = { Body(SN_MOTION_STATIC, 0, -0.5f, 0, 0), Body(SN_MOTION_STATIC, 0, 3, 0, 1), Body(SN_MOTION_DYNAMIC, 0, 8, 0, 2) };
	bodies[0].flags = SN_BODY_CONTACT_EVENTS;
	bodies[1].flags = SN_BODY_SENSOR | SN_BODY_CONTACT_EVENTS;
	REQUIRE(sn_physics_add_bodies(world, bodies, 3, shapes, 3, nullptr, 0, nullptr) == 0);

	// 逐步收集事件：盒子穿过传感器（开始、结束），落到地面（开始；之后睡下也不报结束，同 Bepu）
	std::vector<sn_contact_event> seen;
	for (int i = 0; i < 180; ++i)
	{
		REQUIRE(sn_physics_step(world, 1.0f / 60.0f, 1) == SN_OK);
		uint32_t count = 0;
		const sn_contact_event *events = sn_physics_contact_events(world, &count);
		seen.insert(seen.end(), events, events + count);
	}
	REQUIRE(seen.size() == 3);
	CHECK((seen[0].type == SN_CONTACT_STARTED && seen[0].body_a == 1 && seen[0].body_b == 2));
	CHECK((seen[1].type == SN_CONTACT_STOPPED && seen[1].body_a == 1 && seen[1].body_b == 2));
	CHECK((seen[2].type == SN_CONTACT_STARTED && seen[2].body_a == 0 && seen[2].body_b == 2));
	// 传感器不挡住盒子：最后停在地面上
	CHECK(sn_physics_positions(world)[7] == doctest::Approx(0.5f).epsilon(0.05));

	// 移除：之后的步进不再报和它有关的事件，控制接口报错
	const int32_t removed[] = { 2 };
	REQUIRE(sn_physics_remove_bodies(world, removed, 1) == SN_OK);
	REQUIRE(sn_physics_remove_bodies(world, removed, 1) == SN_OK);	// 移除过的忽略
	REQUIRE(sn_physics_step(world, 1.0f / 60.0f, 1) == SN_OK);
	uint32_t count = 0;
	sn_physics_contact_events(world, &count);
	CHECK(count == 0);
	CHECK(sn_physics_body_set_gravity(world, 2, 0) == SN_ERROR_INVALID_ARGUMENT);
	CHECK(sn_physics_body_set_contact_events(world, 2, 1) == SN_ERROR_INVALID_ARGUMENT);
	sn_physics_destroy(world);
}

namespace
{
	/// PhysicsSample 的 Constraints 场景：运动学的立方体在 (2, 0, -9)，动态的球在 (-2, 0, -9)，重力 -9。返回世界。
	int32_t ConstraintWorld(bool inCubeDynamic = false)
	{
		sn_physics_settings settings {};
		settings.gravity[1] = -9.0f;
		for (uint32_t &mask : settings.collision_masks)
			mask = 0xffffffffu;
		int32_t world = sn_physics_create(8, 0, &settings);
		sn_shape shapes[2] = { BoxShape(2, 2, 2), {} };
		shapes[1].type = SN_SHAPE_SPHERE;
		shapes[1].rotation[3] = 1.0f;
		shapes[1].size[0] = 1.0f;
		shapes[1].mass = 1.0f;
		sn_body bodies[] = { Body(inCubeDynamic ? SN_MOTION_DYNAMIC : SN_MOTION_KINEMATIC, 2, 0, -9, 0), Body(SN_MOTION_DYNAMIC, -2, 0, -9, 1) };
		bodies[0].flags = bodies[1].flags = SN_BODY_NEVER_SLEEP;
		REQUIRE(sn_physics_add_bodies(world, bodies, 2, shapes, 2, nullptr, 0, nullptr) == 0);
		return world;
	}

	sn_constraint Constraint(uint32_t inType, int32_t inA = 0, int32_t inB = 1)
	{
		sn_constraint c {};
		c.type = inType;
		c.body_a = inA;
		c.body_b = inB;
		c.basis_a[3] = c.basis_b[3] = 1.0f;
		return c;
	}

	/// 测试里用的三维向量（只要减法和长度）
	struct Vec3
	{
		float x, y, z;
		float GetX() const { return x; }
		float GetY() const { return y; }
		float GetZ() const { return z; }
		Vec3 operator-(const Vec3 &inRHS) const { return { x - inRHS.x, y - inRHS.y, z - inRHS.z }; }
		float Length() const { return std::sqrt(x * x + y * y + z * z); }
		Vec3 Normalized() const { float l = Length(); return { x / l, y / l, z / l }; }
	};

	Vec3 PositionOf(int32_t inWorld, int32_t inBody)
	{
		const float *p = sn_physics_positions(inWorld) + inBody * 3;
		return { p[0], p[1], p[2] };
	}
}

TEST_CASE("物理：约束（Stride.BepuPhysics 的约束组件，PhysicsSample 的 Constraints 场景）")
{
	ModuleScope module;
	const float impulse_z[3] = { 0, 0, 18 };

	SUBCASE("球窝：球右边 4 米的点停在立方体中心，球绕着它转")
	{
		int32_t world = ConstraintWorld();
		sn_constraint c = Constraint(SN_CONSTRAINT_BALL_SOCKET);
		c.offset_b[0] = 4;
		int32_t id = sn_physics_add_constraint(world, &c);
		REQUIRE(id > 0);
		sn_physics_body_apply_impulse(world, 1, impulse_z, nullptr);
		Step(world, 90);
		CHECK((PositionOf(world, 1) - PositionOf(world, 0)).Length() == doctest::Approx(4.0f).epsilon(0.02));
		// 移除后球自由落下（一秒落下约 4.5 米）
		float height = PositionOf(world, 1).GetY();
		REQUIRE(sn_physics_remove_constraint(world, id) == SN_OK);
		REQUIRE(sn_physics_remove_constraint(world, id) == SN_OK);
		Step(world, 60);
		CHECK(PositionOf(world, 1).GetY() < height - 3.0f);
		sn_physics_destroy(world);
	}

	SUBCASE("铰链：轴是 Z，沿 Z 的冲量被挡住，球只在 XY 平面里摆")
	{
		int32_t world = ConstraintWorld();
		sn_constraint c = Constraint(SN_CONSTRAINT_HINGE);
		c.offset_b[0] = 4;
		c.axis_a[2] = c.axis_b[2] = 1;
		REQUIRE(sn_physics_add_constraint(world, &c) > 0);
		sn_physics_body_apply_impulse(world, 1, impulse_z, nullptr);
		Step(world, 90);
		// 没有铰链时 1.5 秒会沿 Z 走 27 米；刚性求解在大冲量下留一点误差
		CHECK(std::abs(PositionOf(world, 1).GetZ() + 9.0f) < 0.2f);
		CHECK((PositionOf(world, 1) - PositionOf(world, 0)).Length() == doctest::Approx(4.0f).epsilon(0.02));
		sn_physics_destroy(world);
	}

	SUBCASE("齿轮马达：立方体绕 X 的角速度是球的一半，同向")
	{
		int32_t world = ConstraintWorld(true);
		sn_physics_body_set_gravity(world, 0, 0);
		sn_physics_body_set_gravity(world, 1, 0);
		sn_constraint c = Constraint(SN_CONSTRAINT_ANGULAR_AXIS_GEAR_MOTOR, 1, 0);
		c.axis_a[0] = 1;
		c.scale = 0.5f;
		REQUIRE(sn_physics_add_constraint(world, &c) > 0);
		const float spin[3] = { 25, 0, 0 };
		sn_physics_body_set_velocity(world, 1, nullptr, spin);
		Step(world, 30);
		float sphere[3], cube[3], linear[3];
		sn_physics_body_velocity(world, 1, linear, sphere);
		sn_physics_body_velocity(world, 0, linear, cube);
		CHECK(sphere[0] > 1.0f);
		CHECK(cube[0] == doctest::Approx(0.5f * sphere[0]).epsilon(0.05));
		sn_physics_destroy(world);
	}

	SUBCASE("滑轨：球在立方体的 X 轴上，离开不超过左边 4 米")
	{
		int32_t world = ConstraintWorld();
		sn_constraint line = Constraint(SN_CONSTRAINT_POINT_ON_LINE);
		line.axis_a[0] = 1;
		sn_constraint limit = Constraint(SN_CONSTRAINT_LINEAR_AXIS_LIMIT);
		limit.axis_a[0] = 1;
		limit.minimum = -4;
		limit.maximum = 0;
		REQUIRE(sn_physics_add_constraint(world, &line) > 0);
		REQUIRE(sn_physics_add_constraint(world, &limit) > 0);
		const float push[3] = { -25, 0, 0 };
		sn_physics_body_apply_impulse(world, 1, push, nullptr);
		Step(world, 60);
		Vec3 offset = PositionOf(world, 1) - PositionOf(world, 0);
		CHECK(offset.GetY() == doctest::Approx(0.0f).epsilon(0.01));
		CHECK(std::abs(offset.GetZ()) < 0.01f);
		CHECK(offset.GetX() >= -4.05f);
		CHECK(offset.GetX() <= 0.05f);
		sn_physics_destroy(world);
	}

	SUBCASE("锥形扭转：球窝加摆动限位，球的 X 轴偏离立方体的不超过 0.5 弧度")
	{
		int32_t world = ConstraintWorld();
		sn_constraint socket = Constraint(SN_CONSTRAINT_BALL_SOCKET);
		socket.offset_b[0] = 4;
		sn_constraint swing = Constraint(SN_CONSTRAINT_SWING_LIMIT);
		swing.axis_a[0] = swing.axis_b[0] = 1;
		swing.maximum = 0.5f;
		sn_constraint twist = Constraint(SN_CONSTRAINT_TWIST_LIMIT);
		const float s = std::sqrt(0.5f);
		twist.basis_a[1] = twist.basis_b[1] = s;
		twist.basis_a[3] = twist.basis_b[3] = s;
		twist.minimum = -0.5f;
		twist.maximum = 0.5f;
		for (sn_constraint *c : { &socket, &swing, &twist })
			REQUIRE(sn_physics_add_constraint(world, c) > 0);
		sn_physics_body_apply_impulse(world, 1, impulse_z, nullptr);
		Step(world, 120);
		// 球心在立方体中心左边 4 米处的锥里：和 -X 方向的夹角不超过 0.5（加一点误差）
		Vec3 arm = (PositionOf(world, 1) - PositionOf(world, 0)).Normalized();
		CHECK(std::acos(std::clamp(-arm.GetX(), -1.0f, 1.0f)) < 0.55f);
		sn_physics_destroy(world);
	}

	SUBCASE("焊接：球保持在立方体左边 4 米，冲量过后回到原位")
	{
		int32_t world = ConstraintWorld();
		sn_constraint c = Constraint(SN_CONSTRAINT_WELD);
		c.offset_a[0] = -4;
		REQUIRE(sn_physics_add_constraint(world, &c) > 0);
		sn_physics_body_apply_impulse(world, 1, impulse_z, nullptr);
		Step(world, 60);
		Vec3 offset = PositionOf(world, 1) - PositionOf(world, 0);
		CHECK(offset.GetX() == doctest::Approx(-4.0f).epsilon(0.01));
		CHECK(std::abs(offset.GetY()) < 0.05f);
		CHECK(std::abs(offset.GetZ()) < 0.05f);
		// 移除物体时连着的约束一起移除
		const int32_t sphere[] = { 1 };
		REQUIRE(sn_physics_remove_bodies(world, sphere, 1) == SN_OK);
		Step(world, 1);
		sn_physics_destroy(world);
	}

	SUBCASE("参数检查")
	{
		int32_t world = ConstraintWorld();
		sn_constraint same = Constraint(SN_CONSTRAINT_BALL_SOCKET, 1, 1);
		CHECK(sn_physics_add_constraint(world, &same) == SN_ERROR_INVALID_ARGUMENT);
		sn_constraint unknown = Constraint(99);
		CHECK(sn_physics_add_constraint(world, &unknown) == SN_ERROR_INVALID_ARGUMENT);
		CHECK(sn_physics_remove_constraint(world, 5) == SN_ERROR_INVALID_ARGUMENT);
		sn_physics_destroy(world);
	}
}

TEST_CASE("物理：插值的物体在两步之间写回插值的位姿")
{
	ModuleScope module;
	int32_t store = sn_transforms_create(4);
	REQUIRE(store > 0);
	sn_transform_columns c {};
	REQUIRE(sn_transforms_columns(store, &c) == SN_OK);
	int32_t slot = 0;
	REQUIRE(sn_transforms_alloc(store, &slot, 1) == 1);
	c.position[slot * 3 + 1] = 10;

	int32_t world = sn_physics_create(4, store, nullptr);
	REQUIRE(world > 0);
	sn_shape box = BoxShape(1, 1, 1);
	sn_body body = Body(SN_MOTION_DYNAMIC, 0, 0, 0, 0);
	body.transform = slot;
	body.flags = SN_BODY_INTERPOLATED;
	REQUIRE(sn_physics_add_bodies(world, &body, 1, &box, 1, nullptr, 0, nullptr) == 0);
	Step(world, 10);
	float previous = sn_physics_positions(world)[1];
	Step(world, 1);
	float current = sn_physics_positions(world)[1];
	REQUIRE(current < previous);
	CHECK(c.position[slot * 3 + 1] == doctest::Approx(current));
	REQUIRE(sn_physics_interpolate(world, 0.25f) == SN_OK);
	CHECK(c.position[slot * 3 + 1] == doctest::Approx(previous + (current - previous) * 0.25f));
	sn_physics_destroy(world);
	sn_transforms_destroy(store);
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

/// 一个盒子形状的"视锥"：|x| <= 5、|y| <= 5、-20 <= z <= 0，近平面是 z = 0。距离按 -z 量。
static sn_cull_view BoxView(uint32_t inPlaneCount)
{
	sn_cull_view view {};
	const float planes[6][4] = { { 1, 0, 0, 5 }, { -1, 0, 0, 5 }, { 0, 1, 0, 5 }, { 0, -1, 0, 5 }, { 0, 0, -1, 0 }, { 0, 0, 1, 20 } };
	std::memcpy(view.planes, planes, sizeof(planes));
	view.plane_count = inPlaneCount;
	view.measure_distance = 1;
	view.distance_plane[2] = -1;
	return view;
}

TEST_CASE("剔除：包围盒按世界矩阵变换后和视图的平面比较，可见的按批次紧凑写出并量出距离范围")
{
	ModuleScope module;
	int32_t store = sn_transforms_create(4);
	sn_transform_columns c {};
	REQUIRE(sn_transforms_columns(store, &c) == SN_OK);
	int32_t slots[4] = {};
	REQUIRE(sn_transforms_alloc(store, slots, 4) == 4);
	// 第 0 批（单位立方体）：x = -10 在外面；z = +3 在近平面后面；x = 4.8、z = -10 压着边界，算可见
	c.position[slots[0] * 3] = -10;
	c.position[slots[1] * 3 + 2] = 3;
	c.position[slots[2] * 3] = 4.8f;
	c.position[slots[2] * 3 + 2] = -10;
	// 第 1 批的包围盒半长是 0：总是可见，不参与量距离
	c.position[slots[3] * 3] = 100;
	REQUIRE(sn_transforms_update(store) == SN_OK);

	const uint32_t batch_ends[2] = { 3, 4 };
	const float bounds[12] = { 0, 0, 0, 0.5f, 0.5f, 0.5f, 0, 0, 0, 0, 0, 0 };
	// 第二个视图忽略近、远平面（阴影级联的做法）：z = +3 的那个也可见
	sn_cull_view views[2] = { BoxView(6), BoxView(4) };
	int32_t out[8] = {};
	uint32_t counts[4] = {};
	REQUIRE(sn_transforms_cull(store, slots, 4, batch_ends, bounds, 2, views, 2, out, counts) == SN_OK);
	CHECK(counts[0] == 1);
	CHECK(out[0] == slots[2]);
	CHECK(counts[1] == 1);
	CHECK(out[3] == slots[3]);
	CHECK(views[0].min_distance == doctest::Approx(9.5f));
	CHECK(views[0].max_distance == doctest::Approx(10.5f));
	CHECK(counts[2] == 2);
	CHECK((out[4] == slots[1] && out[5] == slots[2]));
	CHECK(views[1].min_distance == doctest::Approx(-3.5f));

	// 旋转的盒子：绕 Y 转 90° 后 x 方向的半长变成 z 方向的
	const float wide[12] = { 0, 0, 0, 8, 0.5f, 0.5f, 0, 0, 0, 0, 0, 0 };
	c.position[slots[0] * 3] = 0;
	c.position[slots[0] * 3 + 2] = -10;
	SetQuarterTurnAroundY(&c.rotation[slots[0] * 4]);
	REQUIRE(sn_transforms_update(store) == SN_OK);
	sn_cull_view rotated = BoxView(6);
	REQUIRE(sn_transforms_cull(store, slots, 4, batch_ends, wide, 2, &rotated, 1, out, counts) == SN_OK);
	CHECK(rotated.min_distance == doctest::Approx(2.0f));
	CHECK(rotated.max_distance == doctest::Approx(18.0f));

	const int32_t free_slot = 99;
	CHECK(sn_transforms_cull(store, &free_slot, 1, batch_ends, bounds, 1, views, 1, out, counts) == SN_ERROR_INVALID_ARGUMENT);
	sn_transforms_destroy(store);
}

TEST_CASE("剔除：实例多时分到多个线程上，结果和单线程一样")
{
	ModuleScope module;
	const uint32_t count = 6000;
	int32_t store = sn_transforms_create(count);
	sn_transform_columns c {};
	REQUIRE(sn_transforms_columns(store, &c) == SN_OK);
	std::vector<int32_t> slots(count);
	REQUIRE(sn_transforms_alloc(store, slots.data(), count) == int32_t(count));
	// 一半在视图里（z = -1），一半在右边外面（x = 50）
	for (uint32_t i = 0; i < count; ++i)
	{
		c.position[slots[i] * 3] = i % 2 == 0 ? 0.0f : 50.0f;
		c.position[slots[i] * 3 + 2] = -1;
	}
	REQUIRE(sn_transforms_update(store) == SN_OK);
	std::vector<uint32_t> batch_ends;
	std::vector<float> bounds;
	for (uint32_t end = 100; end <= count; end += 100)
	{
		batch_ends.push_back(end);
		bounds.insert(bounds.end(), { 0, 0, 0, 0.5f, 0.5f, 0.5f });
	}
	sn_cull_view views[2] = { BoxView(6), BoxView(6) };
	std::vector<int32_t> out(count * 2);
	std::vector<uint32_t> counts(batch_ends.size() * 2);
	REQUIRE(sn_transforms_cull(store, slots.data(), count, batch_ends.data(), bounds.data(), uint32_t(batch_ends.size()), views, 2, out.data(), counts.data()) == SN_OK);
	for (uint32_t b = 0; b < counts.size(); ++b)
		CHECK(counts[b] == 50);
	CHECK(out[0] == slots[0]);
	CHECK(out[1] == slots[2]);
	CHECK(views[1].min_distance == doctest::Approx(0.5f));
	CHECK(views[1].max_distance == doctest::Approx(1.5f));
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

	int32_t world = sn_physics_create(8, store, nullptr);
	REQUIRE(world > 0);
	TestBox root_box = DynamicBox(0, 0, 0), child_box = DynamicBox(0, 0, 0);
	root_box.transform = root;
	child_box.transform = child;
	const TestBox boxes[] = { cFloor, root_box, child_box };
	REQUIRE(AddBoxes(world, boxes, 3) == 0);
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

	TestBox unallocated = DynamicBox(0, 1, 0);
	unallocated.transform = 7;
	CHECK(AddBoxes(world, &unallocated, 1) == SN_ERROR_INVALID_ARGUMENT);
	int32_t plain = sn_physics_create(2, 0, nullptr);
	CHECK(AddBoxes(plain, &root_box, 1) == SN_ERROR_INVALID_ARGUMENT);	// 世界没有变换存储
	CHECK(sn_physics_create(2, 99, nullptr) == SN_ERROR_INVALID_HANDLE);
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

TEST_CASE("Debug counters, timings and log lines reach the JSON snapshot and drain")
{
	ModuleScope module;
	sn_debug_set_enabled(0);
	sn_debug_reset_timings();
	sn_debug_drain_log();
	// 关着时：计数器和普通日志不记，错误照样排队（调试器连上之前的错误不丢）
	sn_debug_set_counter("test.off", 1);
	sn_debug_log(SN_LOG_INFO, "ignored");
	sn_debug_log(SN_LOG_ERROR, "kept \"quoted\"\n");
	CHECK(std::string(sn_debug_snapshot()).find("test.off") == std::string::npos);
	CHECK(std::string(sn_debug_drain_log()) == R"([{"level":3,"message":"kept \"quoted\"\n"}])");
	CHECK(std::string(sn_debug_drain_log()) == "[]");

	sn_debug_set_enabled(1);
	sn_debug_set_counter("test.value", 2.5);
	sn_debug_add_timing("test.timing", 1.0);
	sn_debug_add_timing("test.timing", 3.0);
	int32_t world = sn_physics_create(16, 0, nullptr);
	REQUIRE(world > 0);
	const TestBox boxes[] = { cFloor, DynamicBox(0, 5, 0) };
	REQUIRE(AddBoxes(world, boxes, 2) == 0);
	REQUIRE(sn_physics_step(world, 1.0f / 60.0f, 1) == SN_OK);
	sn_physics_destroy(world);

	std::string snapshot = sn_debug_snapshot();
	CHECK(snapshot.find(R"("test.value":2.5)") != std::string::npos);
	CHECK(snapshot.find(R"("test.timing":{"calls":2,"ms":4,"maxMs":3})") != std::string::npos);
	CHECK(snapshot.find(R"("physics.bodies":2)") != std::string::npos);
	CHECK(snapshot.find(R"("physics.step":{"calls":1,)") != std::string::npos);
	sn_debug_reset_timings();
	CHECK(std::string(sn_debug_snapshot()).find("physics.step") == std::string::npos);
	sn_debug_set_enabled(0);
}
