// Navmeshes (Detour) and crowds (DetourCrowd). Building runs as a parallel task; queries and crowd
// updates are synchronous and meant for the thread that owns the navmesh (the game loop).
#include <Core/Core.h>
#include <Core/HandleTable.h>
#include <Core/Tasks.h>
#include <Shared/NavMeshBuilder.h>
#include <StrideNative/Navigation.h>
#include <StrideNative/Plugin.h>

#include <DetourCrowd.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

SN_PLUGIN(navigation)

namespace
{
	using namespace StrideNative;

	constexpr int cMaxQueryNodes = 4096;
	constexpr int cMaxPathPolys = 256;

	struct NavMesh
	{
		dtNavMesh *			mNavMesh = nullptr;
		dtNavMeshQuery *	mQuery = nullptr;		///< Not thread-safe: one per thread that queries
		dtQueryFilter		mFilter;

		~NavMesh()
		{
			dtFreeNavMeshQuery(mQuery);
			dtFreeNavMesh(mNavMesh);
		}
	};

	struct Crowd
	{
		dtCrowd *			mCrowd = nullptr;
		int32_t				mNavMesh = 0;			///< The navmesh must outlive the crowd
		std::vector<float>	mPositions;				///< xyz per agent slot, NaN when empty; sized once so callers can keep a view

		~Crowd() { dtFreeCrowd(mCrowd); }

		void WritePosition(int inAgent)
		{
			const dtCrowdAgent *agent = mCrowd->getAgent(inAgent);
			for (int axis = 0; axis < 3; ++axis)
				mPositions[inAgent * 3 + axis] = agent->active ? agent->npos[axis] : std::numeric_limits<float>::quiet_NaN();
		}
	};

	int32_t AddNavMesh(dtNavMesh *inNavMesh, std::string &outError)
	{
		auto nav = std::make_unique<NavMesh>();
		nav->mNavMesh = inNavMesh;
		nav->mQuery = dtAllocNavMeshQuery();
		if (nav->mQuery == nullptr || dtStatusFailed(nav->mQuery->init(inNavMesh, cMaxQueryNodes)))
		{
			outError = "dtNavMeshQuery::init failed";
			return SN_ERROR_FAILED;
		}
		return GetTable<NavMesh>().Add(std::move(nav));
	}
}

SN_API int32_t sn_nav_build(const float *vertices, uint32_t vertex_count, const int32_t *triangles, uint32_t triangle_count, const sn_nav_config *config)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (config == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "config is null");

	auto builder = std::make_shared<NavMeshBuilder>();
	std::string error;
	if (!builder->Prepare(vertices, vertex_count, triangles, triangle_count, *config, error))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, error);

	return RunParallelTask("NavMeshBuild", builder->GetTileCount(), [builder](Task &ioTask, uint32_t inTile)
	{
		std::string tile_error;
		if (!builder->BuildTile(inTile, tile_error))
			ioTask.Fail(tile_error);
	},
	[builder](Task &ioTask)
	{
		std::string create_error;
		dtNavMesh *nav_mesh = builder->CreateNavMesh(create_error);
		int32_t handle = nav_mesh != nullptr ? AddNavMesh(nav_mesh, create_error) : SN_ERROR_FAILED;
		if (handle < 0)
		{
			ioTask.Fail(create_error);
			return false;
		}
		ioTask.SetResult(0, handle);
		ioTask.SetResult(1, int32_t(builder->GetTileCount()));
		return true;
	});
}

SN_API int32_t sn_nav_load(const void *data, uint32_t size)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	std::string error;
	dtNavMesh *nav_mesh = DeserializeNavMesh(data, size, error);
	if (nav_mesh == nullptr)
		return SetLastError(SN_ERROR_FAILED, error);
	int32_t handle = AddNavMesh(nav_mesh, error);
	return handle < 0 ? SetLastError(handle, error) : handle;
}

SN_API int32_t sn_nav_serialize(int32_t navmesh, void **out_data, uint32_t *out_size)
{
	NavMesh *nav = Lookup<NavMesh>(navmesh, "navmesh");
	if (nav == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (out_data == nullptr || out_size == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "out_data and out_size are required");

	std::vector<uint8_t> bytes;
	SerializeNavMesh(*nav->mNavMesh, bytes);
	void *copy = sn_alloc(uint32_t(bytes.size()));
	std::memcpy(copy, bytes.data(), bytes.size());
	*out_data = copy;
	*out_size = uint32_t(bytes.size());
	return SN_OK;
}

SN_API void sn_nav_destroy(int32_t navmesh)
{
	GetTable<NavMesh>().Remove(navmesh);
}

SN_API int32_t sn_nav_find_path(int32_t navmesh, const float *start3, const float *end3, const float *search_extents3, float *out_points, uint32_t max_points)
{
	NavMesh *nav = Lookup<NavMesh>(navmesh, "navmesh");
	if (nav == nullptr)
		return SN_ERROR_INVALID_HANDLE;
	if (start3 == nullptr || end3 == nullptr || search_extents3 == nullptr || out_points == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "start, end, extents and output are required");

	dtPolyRef start_ref = 0, end_ref = 0;
	float start_point[3], end_point[3];
	nav->mQuery->findNearestPoly(start3, search_extents3, &nav->mFilter, &start_ref, start_point);
	nav->mQuery->findNearestPoly(end3, search_extents3, &nav->mFilter, &end_ref, end_point);
	if (start_ref == 0 || end_ref == 0)
		return SetLastError(SN_ERROR_FAILED, "start or end is not on the navmesh");

	dtPolyRef polys[cMaxPathPolys];
	int poly_count = 0;
	if (dtStatusFailed(nav->mQuery->findPath(start_ref, end_ref, start_point, end_point, &nav->mFilter, polys, &poly_count, cMaxPathPolys)) || poly_count == 0)
		return SetLastError(SN_ERROR_FAILED, "no path found");

	// A partial path ends at the closest reachable polygon; aim for the closest point on it.
	float target[3] = { end_point[0], end_point[1], end_point[2] };
	if (polys[poly_count - 1] != end_ref)
		nav->mQuery->closestPointOnPoly(polys[poly_count - 1], end_point, target, nullptr);

	int point_count = 0;
	if (dtStatusFailed(nav->mQuery->findStraightPath(start_point, target, polys, poly_count, out_points, nullptr, nullptr, &point_count, int(max_points))))
		return SetLastError(SN_ERROR_FAILED, "findStraightPath failed");
	return point_count;
}

SN_API int32_t sn_crowd_create(int32_t navmesh, uint32_t max_agents, float max_agent_radius)
{
	NavMesh *nav = Lookup<NavMesh>(navmesh, "navmesh");
	if (nav == nullptr)
		return SN_ERROR_INVALID_HANDLE;

	auto crowd = std::make_unique<Crowd>();
	crowd->mNavMesh = navmesh;
	crowd->mCrowd = dtAllocCrowd();
	if (crowd->mCrowd == nullptr || !crowd->mCrowd->init(int(max_agents), max_agent_radius, nav->mNavMesh))
		return SetLastError(SN_ERROR_FAILED, "dtCrowd::init failed");
	crowd->mPositions.assign(size_t(max_agents) * 3, std::numeric_limits<float>::quiet_NaN());
	return GetTable<Crowd>().Add(std::move(crowd));
}

SN_API void sn_crowd_destroy(int32_t crowd)
{
	GetTable<Crowd>().Remove(crowd);
}

SN_API int32_t sn_crowd_add_agent(int32_t crowd, const float *position3, float radius, float height, float max_speed, float max_acceleration)
{
	Crowd *c = Lookup<Crowd>(crowd, "crowd");
	if (c == nullptr)
		return SN_ERROR_INVALID_HANDLE;

	dtCrowdAgentParams params {};
	params.radius = radius;
	params.height = height;
	params.maxSpeed = max_speed;
	params.maxAcceleration = max_acceleration;
	params.collisionQueryRange = radius * 12.0f;
	params.pathOptimizationRange = radius * 30.0f;
	params.updateFlags = DT_CROWD_ANTICIPATE_TURNS | DT_CROWD_OPTIMIZE_VIS | DT_CROWD_OPTIMIZE_TOPO | DT_CROWD_OBSTACLE_AVOIDANCE | DT_CROWD_SEPARATION;
	params.separationWeight = 2.0f;
	int agent = c->mCrowd->addAgent(position3, &params);
	if (agent < 0)
		return SetLastError(SN_ERROR_FAILED, "crowd is full or position is off the navmesh");
	c->WritePosition(agent);
	return agent;
}

SN_API void sn_crowd_remove_agent(int32_t crowd, int32_t agent)
{
	Crowd *c = Lookup<Crowd>(crowd, "crowd");
	if (c == nullptr || agent < 0 || agent >= c->mCrowd->getAgentCount())
		return;
	c->mCrowd->removeAgent(agent);
	c->WritePosition(agent);
}

SN_API int32_t sn_crowd_request_move(int32_t crowd, int32_t agent, const float *target3)
{
	Crowd *c = Lookup<Crowd>(crowd, "crowd");
	NavMesh *nav = c != nullptr ? Lookup<NavMesh>(c->mNavMesh, "navmesh") : nullptr;
	if (nav == nullptr)
		return SN_ERROR_INVALID_HANDLE;

	const dtQueryFilter *filter = c->mCrowd->getFilter(0);
	const float *extents = c->mCrowd->getQueryExtents();
	dtPolyRef target_ref = 0;
	float target_point[3];
	nav->mQuery->findNearestPoly(target3, extents, filter, &target_ref, target_point);
	if (target_ref == 0 || !c->mCrowd->requestMoveTarget(agent, target_ref, target_point))
		return SetLastError(SN_ERROR_FAILED, "move target is not on the navmesh");
	return SN_OK;
}

SN_API void sn_crowd_update(int32_t crowd, float delta_time)
{
	Crowd *c = Lookup<Crowd>(crowd, "crowd");
	if (c == nullptr)
		return;
	c->mCrowd->update(delta_time, nullptr);
	for (int agent = 0; agent < c->mCrowd->getAgentCount(); ++agent)
		c->WritePosition(agent);
}

SN_API const float *sn_crowd_positions(int32_t crowd)
{
	Crowd *c = Lookup<Crowd>(crowd, "crowd");
	return c != nullptr ? c->mPositions.data() : nullptr;
}
