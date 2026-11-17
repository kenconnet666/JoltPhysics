// Offline navmesh bake: the runtime's tiled parallel build, serialized for sn_nav_load.
#include <Core/Core.h>
#include <Core/Tasks.h>
#include <Shared/NavMeshBuilder.h>
#include <StrideNative/Tools.h>

#include <DetourNavMesh.h>

#include <cstring>
#include <memory>

using namespace StrideNative;

SN_API int32_t st_nav_bake(const float *vertices, uint32_t vertex_count, const int32_t *triangles, uint32_t triangle_count, const sn_nav_config *config)
{
	SN_REQUIRE_INIT(SN_ERROR_NOT_INITIALIZED);
	if (config == nullptr)
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, "config is null");

	auto builder = std::make_shared<NavMeshBuilder>();
	std::string error;
	if (!builder->Prepare(vertices, vertex_count, triangles, triangle_count, *config, error))
		return SetLastError(SN_ERROR_INVALID_ARGUMENT, error);

	return RunParallelTask("NavMeshBake", builder->GetTileCount(), [builder](Task &ioTask, uint32_t inTile)
	{
		std::string tile_error;
		if (!builder->BuildTile(inTile, tile_error))
			ioTask.Fail(tile_error);
	},
	[builder](Task &ioTask)
	{
		std::string create_error;
		std::unique_ptr<dtNavMesh, decltype(&dtFreeNavMesh)> nav_mesh(builder->CreateNavMesh(create_error), &dtFreeNavMesh);
		if (nav_mesh == nullptr)
		{
			ioTask.Fail(create_error);
			return false;
		}
		std::vector<uint8_t> data;
		SerializeNavMesh(*nav_mesh, data);
		std::memcpy(ioTask.AddOutput(uint32_t(data.size())), data.data(), data.size());
		ioTask.SetResult(0, int32_t(builder->GetTileCount()));
		return true;
	});
}
