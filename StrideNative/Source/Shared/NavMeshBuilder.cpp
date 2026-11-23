// Follows RecastDemo's Sample_TileMesh::buildTileMesh, minus the debug output.
#include <Shared/NavMeshBuilder.h>
#include <Core/Core.h>

#include <DetourAlloc.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <Recast.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <memory>
#include <unordered_map>

namespace StrideNative
{
	namespace
	{
		constexpr uint32_t cNavMagic = 'S' << 24 | 'N' << 16 | 'N' << 8 | 'V';
		constexpr uint32_t cNavVersion = 2;	///< 2: 64-bit tile refs (DT_POLYREF64)
		constexpr int cMaxPolysPerTile = 1 << 16;
		constexpr unsigned short cWalkFlag = 1;

		struct NavFileHeader
		{
			uint32_t			mMagic;
			uint32_t			mVersion;
			uint32_t			mTileCount;
			dtNavMeshParams		mParams;
		};

		struct NavTileHeader
		{
			dtTileRef			mRef;
			int32_t				mDataSize;
		};

		struct RecastDeleter
		{
			void operator()(rcHeightfield *inPtr) const { rcFreeHeightField(inPtr); }
			void operator()(rcCompactHeightfield *inPtr) const { rcFreeCompactHeightfield(inPtr); }
			void operator()(rcContourSet *inPtr) const { rcFreeContourSet(inPtr); }
			void operator()(rcPolyMesh *inPtr) const { rcFreePolyMesh(inPtr); }
			void operator()(rcPolyMeshDetail *inPtr) const { rcFreePolyMeshDetail(inPtr); }
		};

		template <class T>
		using RecastPtr = std::unique_ptr<T, RecastDeleter>;

		int NextPowerOfTwo(int inValue)
		{
			int result = 1;
			while (result < inValue)
				result <<= 1;
			return result;
		}

	}

	NavMeshBuilder::~NavMeshBuilder()
	{
		for (Tile &tile : mTiles)
			if (tile.mData != nullptr)
				dtFree(tile.mData);
	}

	bool NavMeshBuilder::Validate(const float *inVertices, uint32_t inVertexCount, const int32_t *inTriangles, uint32_t inTriangleCount, const sn_nav_config &inConfig, std::string &outError)
	{
		if (inVertices == nullptr || inTriangles == nullptr || inVertexCount == 0 || inTriangleCount == 0)
		{
			outError = "navmesh input has no geometry";
			return false;
		}
		if (inConfig.cell_size <= 0.0f || inConfig.cell_height <= 0.0f || inConfig.tile_size <= 0)
		{
			outError = "cell_size, cell_height and tile_size must be positive";
			return false;
		}
		for (uint32_t i = 0; i < inTriangleCount * 3; ++i)
			if (inTriangles[i] < 0 || uint32_t(inTriangles[i]) >= inVertexCount)
			{
				outError = "triangle index " + std::to_string(inTriangles[i]) + " is out of range";
				return false;
			}
		mVertices = inVertices;
		mVertexCount = inVertexCount;
		mConfig = inConfig;
		rcCalcBounds(inVertices, int(inVertexCount), mBoundsMin, mBoundsMax);
		return true;
	}

	bool NavMeshBuilder::Prepare(const float *inVertices, uint32_t inVertexCount, const int32_t *inTriangles, uint32_t inTriangleCount, const sn_nav_config &inConfig, std::string &outError)
	{
		if (!Validate(inVertices, inVertexCount, inTriangles, inTriangleCount, inConfig, outError))
			return false;
		rcVcopy(mOrigin, mBoundsMin);

		int grid_width = 0, grid_height = 0;
		rcCalcGridSize(mBoundsMin, mBoundsMax, inConfig.cell_size, &grid_width, &grid_height);
		mTilesX = (grid_width + inConfig.tile_size - 1) / inConfig.tile_size;
		mTilesY = (grid_height + inConfig.tile_size - 1) / inConfig.tile_size;
		if (mTilesX * mTilesY > (1 << 16))
		{
			outError = "too many navmesh tiles; raise tile_size or cell_size";
			return false;
		}

		mTiles.resize(size_t(mTilesX) * mTilesY);
		for (int y = 0; y < mTilesY; ++y)
			for (int x = 0; x < mTilesX; ++x)
			{
				Tile &tile = mTiles[size_t(y) * mTilesX + x];
				tile.mX = x;
				tile.mY = y;
			}
		BinTriangles(inTriangles, inTriangleCount);
		return true;
	}

	bool NavMeshBuilder::PrepareTiles(const float *inVertices, uint32_t inVertexCount, const int32_t *inTriangles, uint32_t inTriangleCount, const sn_nav_config &inConfig,
		const float inOrigin[3], const int32_t *inTilesXY, uint32_t inTileCount, std::string &outError)
	{
		if (inTilesXY == nullptr && inTileCount > 0)
		{
			outError = "tiles are required";
			return false;
		}
		if (inTriangleCount > 0 && !Validate(inVertices, inVertexCount, inTriangles, inTriangleCount, inConfig, outError))
			return false;
		mConfig = inConfig;
		rcVcopy(mOrigin, inOrigin);
		mTiles.resize(inTileCount);
		for (uint32_t i = 0; i < inTileCount; ++i)
		{
			mTiles[i].mX = inTilesXY[i * 2];
			mTiles[i].mY = inTilesXY[i * 2 + 1];
		}
		if (inTriangleCount > 0)
			BinTriangles(inTriangles, inTriangleCount);
		return true;
	}

	void NavMeshBuilder::BinTriangles(const int32_t *inTriangles, uint32_t inTriangleCount)
	{
		const float tile_world_size = mConfig.tile_size * mConfig.cell_size;
		const int border_cells = int(std::ceil(mConfig.agent_radius / mConfig.cell_size)) + 3;
		const float border = border_cells * mConfig.cell_size;
		// 整个网格（Prepare）按下标直接找；指定的块（PrepareTiles）按坐标查表
		const bool grid = mTilesX > 0;
		std::unordered_map<int64_t, uint32_t> listed;
		if (!grid)
			for (uint32_t i = 0; i < mTiles.size(); ++i)
				listed.emplace(int64_t(mTiles[i].mX) << 32 | uint32_t(mTiles[i].mY), i);
		for (uint32_t t = 0; t < inTriangleCount; ++t)
		{
			const int32_t *tri = inTriangles + t * 3;
			float min_x = mVertices[tri[0] * 3], max_x = min_x, min_z = mVertices[tri[0] * 3 + 2], max_z = min_z;
			for (int corner = 1; corner < 3; ++corner)
			{
				min_x = std::min(min_x, mVertices[tri[corner] * 3]);
				max_x = std::max(max_x, mVertices[tri[corner] * 3]);
				min_z = std::min(min_z, mVertices[tri[corner] * 3 + 2]);
				max_z = std::max(max_z, mVertices[tri[corner] * 3 + 2]);
			}
			int x0 = int(std::floor((min_x - border - mOrigin[0]) / tile_world_size));
			int x1 = int(std::floor((max_x + border - mOrigin[0]) / tile_world_size));
			int y0 = int(std::floor((min_z - border - mOrigin[2]) / tile_world_size));
			int y1 = int(std::floor((max_z + border - mOrigin[2]) / tile_world_size));
			if (grid)
			{
				x0 = std::max(0, x0);
				x1 = std::min(mTilesX - 1, x1);
				y0 = std::max(0, y0);
				y1 = std::min(mTilesY - 1, y1);
			}
			for (int y = y0; y <= y1; ++y)
				for (int x = x0; x <= x1; ++x)
				{
					uint32_t index;
					if (grid)
						index = uint32_t(y) * mTilesX + x;
					else
					{
						auto found = listed.find(int64_t(x) << 32 | uint32_t(y));
						if (found == listed.end())
							continue;
						index = found->second;
					}
					std::vector<int32_t> &tris = mTiles[index].mTriangles;
					tris.insert(tris.end(), tri, tri + 3);
				}
		}
	}

	bool NavMeshBuilder::BuildTile(uint32_t inTileIndex, std::string &outError)
	{
		Tile &tile = mTiles[inTileIndex];
		if (tile.mTriangles.empty())
			return true;

		const sn_nav_config &c = mConfig;
		rcConfig cfg {};
		cfg.cs = c.cell_size;
		cfg.ch = c.cell_height;
		cfg.walkableSlopeAngle = c.agent_max_slope_degrees;
		cfg.walkableHeight = int(std::ceil(c.agent_height / cfg.ch));
		cfg.walkableClimb = int(std::floor(c.agent_max_climb / cfg.ch));
		cfg.walkableRadius = int(std::ceil(c.agent_radius / cfg.cs));
		cfg.maxEdgeLen = int(c.max_edge_length / cfg.cs);
		cfg.maxSimplificationError = c.max_simplification_error;
		cfg.minRegionArea = int(c.min_region_size * c.min_region_size);
		cfg.mergeRegionArea = int(c.merge_region_size * c.merge_region_size);
		cfg.maxVertsPerPoly = DT_VERTS_PER_POLYGON;
		cfg.tileSize = c.tile_size;
		cfg.borderSize = cfg.walkableRadius + 3;
		cfg.width = cfg.tileSize + cfg.borderSize * 2;
		cfg.height = cfg.tileSize + cfg.borderSize * 2;
		cfg.detailSampleDist = c.detail_sample_distance < 0.9f ? 0.0f : cfg.cs * c.detail_sample_distance;
		cfg.detailSampleMaxError = cfg.ch * c.detail_sample_max_error;

		const float tile_world_size = cfg.tileSize * cfg.cs;
		// 高度的范围按这一块自己的三角形算：Recast 的高度场最多 8191 格高（格高 0.2 米时 1.6 公里），按整个输入算时山区、或者
		// 一批里有一个很深很高的静态物体，地面就落到范围外、整块没有可走的地方
		float min_y = FLT_MAX, max_y = -FLT_MAX;
		for (int32_t vertex : tile.mTriangles)
		{
			min_y = std::min(min_y, mVertices[vertex * 3 + 1]);
			max_y = std::max(max_y, mVertices[vertex * 3 + 1]);
		}
		// 这一块自己还是超出时留最上面的那一段（走的地面在上面，往下很深的东西——地基、深坑的底——切掉）
		cfg.bmin[1] = std::max(min_y, max_y - (RC_SPAN_MAX_HEIGHT - 1) * cfg.ch);
		cfg.bmax[1] = max_y;
		cfg.bmin[0] = mOrigin[0] + tile.mX * tile_world_size - cfg.borderSize * cfg.cs;
		cfg.bmin[2] = mOrigin[2] + tile.mY * tile_world_size - cfg.borderSize * cfg.cs;
		cfg.bmax[0] = mOrigin[0] + (tile.mX + 1) * tile_world_size + cfg.borderSize * cfg.cs;
		cfg.bmax[2] = mOrigin[2] + (tile.mY + 1) * tile_world_size + cfg.borderSize * cfg.cs;

		rcContext context(false);
		auto fail = [&](const char *inStep)
		{
			outError = std::string(inStep) + " failed for tile " + std::to_string(tile.mX) + "," + std::to_string(tile.mY);
			return false;
		};

		RecastPtr<rcHeightfield> solid(rcAllocHeightfield());
		if (!solid || !rcCreateHeightfield(&context, *solid, cfg.width, cfg.height, cfg.bmin, cfg.bmax, cfg.cs, cfg.ch))
			return fail("rcCreateHeightfield");

		int triangle_count = int(tile.mTriangles.size() / 3);
		std::vector<unsigned char> areas(triangle_count, 0);
		rcMarkWalkableTriangles(&context, cfg.walkableSlopeAngle, mVertices, int(mVertexCount), tile.mTriangles.data(), triangle_count, areas.data());
		if (!rcRasterizeTriangles(&context, mVertices, int(mVertexCount), tile.mTriangles.data(), areas.data(), triangle_count, *solid, cfg.walkableClimb))
			return fail("rcRasterizeTriangles");

		rcFilterLowHangingWalkableObstacles(&context, cfg.walkableClimb, *solid);
		rcFilterLedgeSpans(&context, cfg.walkableHeight, cfg.walkableClimb, *solid);
		rcFilterWalkableLowHeightSpans(&context, cfg.walkableHeight, *solid);

		RecastPtr<rcCompactHeightfield> compact(rcAllocCompactHeightfield());
		if (!compact || !rcBuildCompactHeightfield(&context, cfg.walkableHeight, cfg.walkableClimb, *solid, *compact))
			return fail("rcBuildCompactHeightfield");
		solid.reset();

		if (!rcErodeWalkableArea(&context, cfg.walkableRadius, *compact))
			return fail("rcErodeWalkableArea");
		if (!rcBuildDistanceField(&context, *compact))
			return fail("rcBuildDistanceField");
		if (!rcBuildRegions(&context, *compact, cfg.borderSize, cfg.minRegionArea, cfg.mergeRegionArea))
			return fail("rcBuildRegions");

		RecastPtr<rcContourSet> contours(rcAllocContourSet());
		if (!contours || !rcBuildContours(&context, *compact, cfg.maxSimplificationError, cfg.maxEdgeLen, *contours))
			return fail("rcBuildContours");
		if (contours->nconts == 0)
			return true;

		RecastPtr<rcPolyMesh> poly_mesh(rcAllocPolyMesh());
		if (!poly_mesh || !rcBuildPolyMesh(&context, *contours, cfg.maxVertsPerPoly, *poly_mesh))
			return fail("rcBuildPolyMesh");
		RecastPtr<rcPolyMeshDetail> detail_mesh(rcAllocPolyMeshDetail());
		if (!detail_mesh || !rcBuildPolyMeshDetail(&context, *poly_mesh, *compact, cfg.detailSampleDist, cfg.detailSampleMaxError, *detail_mesh))
			return fail("rcBuildPolyMeshDetail");
		if (poly_mesh->npolys == 0)
			return true;

		for (int i = 0; i < poly_mesh->npolys; ++i)
		{
			if (poly_mesh->areas[i] == RC_WALKABLE_AREA)
				poly_mesh->areas[i] = 0;
			poly_mesh->flags[i] = cWalkFlag;
		}

		dtNavMeshCreateParams params {};
		params.verts = poly_mesh->verts;
		params.vertCount = poly_mesh->nverts;
		params.polys = poly_mesh->polys;
		params.polyAreas = poly_mesh->areas;
		params.polyFlags = poly_mesh->flags;
		params.polyCount = poly_mesh->npolys;
		params.nvp = poly_mesh->nvp;
		params.detailMeshes = detail_mesh->meshes;
		params.detailVerts = detail_mesh->verts;
		params.detailVertsCount = detail_mesh->nverts;
		params.detailTris = detail_mesh->tris;
		params.detailTriCount = detail_mesh->ntris;
		params.walkableHeight = c.agent_height;
		params.walkableRadius = c.agent_radius;
		params.walkableClimb = c.agent_max_climb;
		params.tileX = tile.mX;
		params.tileY = tile.mY;
		params.tileLayer = 0;
		rcVcopy(params.bmin, poly_mesh->bmin);
		rcVcopy(params.bmax, poly_mesh->bmax);
		params.cs = cfg.cs;
		params.ch = cfg.ch;
		params.buildBvTree = true;
		if (!dtCreateNavMeshData(&params, &tile.mData, &tile.mDataSize))
			return fail("dtCreateNavMeshData");
		return true;
	}

	dtNavMesh *NavMeshBuilder::CreateNavMesh(std::string &outError)
	{
		dtNavMeshParams params {};
		rcVcopy(params.orig, mOrigin);
		params.tileWidth = mConfig.tile_size * mConfig.cell_size;
		params.tileHeight = mConfig.tile_size * mConfig.cell_size;
		params.maxTiles = NextPowerOfTwo(std::max(1, mTilesX * mTilesY));
		params.maxPolys = cMaxPolysPerTile;

		dtNavMesh *nav_mesh = dtAllocNavMesh();
		if (nav_mesh == nullptr || dtStatusFailed(nav_mesh->init(&params)))
		{
			dtFreeNavMesh(nav_mesh);
			outError = "dtNavMesh::init failed";
			return nullptr;
		}
		for (Tile &tile : mTiles)
		{
			if (tile.mData == nullptr)
				continue;
			if (dtStatusFailed(nav_mesh->addTile(tile.mData, tile.mDataSize, DT_TILE_FREE_DATA, 0, nullptr)))
			{
				dtFreeNavMesh(nav_mesh);
				outError = "dtNavMesh::addTile failed";
				return nullptr;
			}
			tile.mData = nullptr; // Owned by the navmesh now
		}
		return nav_mesh;
	}

	int NavMeshBuilder::ReplaceTiles(dtNavMesh &ioNavMesh, std::string &outError)
	{
		int with_data = 0;
		for (Tile &tile : mTiles)
		{
			if (dtTileRef old = ioNavMesh.getTileRefAt(tile.mX, tile.mY, 0))
				ioNavMesh.removeTile(old, nullptr, nullptr);
			if (tile.mData == nullptr)
				continue;
			if (dtStatusFailed(ioNavMesh.addTile(tile.mData, tile.mDataSize, DT_TILE_FREE_DATA, 0, nullptr)))
			{
				outError = "dtNavMesh::addTile failed for tile " + std::to_string(tile.mX) + "," + std::to_string(tile.mY) + " (max_tiles reached?)";
				return -1;
			}
			tile.mData = nullptr; // Owned by the navmesh now
			++with_data;
		}
		return with_data;
	}

	dtNavMesh *CreateTiledNavMesh(const sn_nav_config &inConfig, uint32_t inMaxTiles, std::string &outError)
	{
		if (inConfig.cell_size <= 0.0f || inConfig.cell_height <= 0.0f || inConfig.tile_size <= 0)
		{
			outError = "cell_size, cell_height and tile_size must be positive";
			return nullptr;
		}
		if (inMaxTiles == 0 || inMaxTiles > (1u << 20))
		{
			outError = "max_tiles must be 1..1048576";
			return nullptr;
		}
		dtNavMeshParams params {};
		params.tileWidth = inConfig.tile_size * inConfig.cell_size;
		params.tileHeight = inConfig.tile_size * inConfig.cell_size;
		params.maxTiles = NextPowerOfTwo(int(inMaxTiles));
		params.maxPolys = cMaxPolysPerTile;
		dtNavMesh *nav_mesh = dtAllocNavMesh();
		if (nav_mesh == nullptr || dtStatusFailed(nav_mesh->init(&params)))
		{
			dtFreeNavMesh(nav_mesh);
			outError = "dtNavMesh::init failed";
			return nullptr;
		}
		return nav_mesh;
	}

	void DefaultNavConfig(sn_nav_config &outConfig)
	{
		outConfig.cell_size = 0.3f;
		outConfig.cell_height = 0.2f;
		outConfig.agent_height = 2.0f;
		outConfig.agent_radius = 0.6f;
		outConfig.agent_max_climb = 0.9f;
		outConfig.agent_max_slope_degrees = 45.0f;
		outConfig.tile_size = 48;
		outConfig.max_edge_length = 12.0f;
		outConfig.max_simplification_error = 1.3f;
		outConfig.min_region_size = 8.0f;
		outConfig.merge_region_size = 20.0f;
		outConfig.detail_sample_distance = 6.0f;
		outConfig.detail_sample_max_error = 1.0f;
	}

	void SerializeNavMesh(const dtNavMesh &inNavMesh, std::vector<uint8_t> &outData)
	{
		NavFileHeader header {};
		header.mMagic = cNavMagic;
		header.mVersion = cNavVersion;
		header.mParams = *inNavMesh.getParams();
		for (int i = 0; i < inNavMesh.getMaxTiles(); ++i)
		{
			const dtMeshTile *tile = inNavMesh.getTile(i);
			if (tile != nullptr && tile->header != nullptr && tile->dataSize > 0)
				++header.mTileCount;
		}

		outData.clear();
		auto append = [&outData](const void *inBytes, size_t inSize)
		{
			const uint8_t *bytes = static_cast<const uint8_t *>(inBytes);
			outData.insert(outData.end(), bytes, bytes + inSize);
		};
		append(&header, sizeof(header));
		for (int i = 0; i < inNavMesh.getMaxTiles(); ++i)
		{
			const dtMeshTile *tile = inNavMesh.getTile(i);
			if (tile == nullptr || tile->header == nullptr || tile->dataSize <= 0)
				continue;
			NavTileHeader tile_header { inNavMesh.getTileRef(tile), tile->dataSize };
			append(&tile_header, sizeof(tile_header));
			append(tile->data, size_t(tile->dataSize));
		}
	}

	dtNavMesh *DeserializeNavMesh(const void *inData, uint32_t inSize, std::string &outError)
	{
		const uint8_t *cursor = static_cast<const uint8_t *>(inData);
		const uint8_t *end = cursor + inSize;
		auto read = [&cursor, end](void *outBytes, size_t inBytes)
		{
			if (size_t(end - cursor) < inBytes)
				return false;
			std::memcpy(outBytes, cursor, inBytes);
			cursor += inBytes;
			return true;
		};

		NavFileHeader header;
		if (!read(&header, sizeof(header)) || header.mMagic != cNavMagic || header.mVersion != cNavVersion)
		{
			outError = "not a stride navmesh (bad header or version)";
			return nullptr;
		}

		dtNavMesh *nav_mesh = dtAllocNavMesh();
		if (nav_mesh == nullptr || dtStatusFailed(nav_mesh->init(&header.mParams)))
		{
			dtFreeNavMesh(nav_mesh);
			outError = "dtNavMesh::init failed";
			return nullptr;
		}
		for (uint32_t i = 0; i < header.mTileCount; ++i)
		{
			NavTileHeader tile_header;
			if (!read(&tile_header, sizeof(tile_header)) || tile_header.mDataSize <= 0 || size_t(end - cursor) < size_t(tile_header.mDataSize))
			{
				dtFreeNavMesh(nav_mesh);
				outError = "navmesh data is truncated";
				return nullptr;
			}
			unsigned char *data = static_cast<unsigned char *>(dtAlloc(tile_header.mDataSize, DT_ALLOC_PERM));
			read(data, size_t(tile_header.mDataSize));
			if (dtStatusFailed(nav_mesh->addTile(data, tile_header.mDataSize, DT_TILE_FREE_DATA, tile_header.mRef, nullptr)))
			{
				dtFree(data);
				dtFreeNavMesh(nav_mesh);
				outError = "dtNavMesh::addTile failed";
				return nullptr;
			}
		}
		return nav_mesh;
	}
}

SN_API void sn_nav_default_config(sn_nav_config *out_config)
{
	if (out_config != nullptr)
		StrideNative::DefaultNavConfig(*out_config);
}
