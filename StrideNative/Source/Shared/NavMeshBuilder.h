// Tiled Recast build shared by the runtime (sn_nav_build) and the pipeline (st_nav_bake).
// Tiles are independent, so BuildTile runs on many threads at once; each call uses its own
// rcContext and scratch data, the input is only read.
#pragma once

#include <StrideNative/Navigation.h>

#include <cstdint>
#include <string>
#include <vector>

class dtNavMesh;

namespace StrideNative
{
	class NavMeshBuilder
	{
	public:
		~NavMeshBuilder();

		/// Validates the input, computes the tile grid and bins triangles into tiles. The input
		/// arrays are referenced, not copied.
		bool Prepare(const float *inVertices, uint32_t inVertexCount, const int32_t *inTriangles, uint32_t inTriangleCount, const sn_nav_config &inConfig, std::string &outError);

		uint32_t GetTileCount() const { return uint32_t(mTiles.size()); }

		/// Builds one tile; tiles without walkable area produce no data. Thread-safe across tiles.
		bool BuildTile(uint32_t inTileIndex, std::string &outError);

		/// Creates the navmesh from the built tiles, which it then owns. Call once, after all tiles.
		dtNavMesh *CreateNavMesh(std::string &outError);

	private:
		struct Tile
		{
			int						mX = 0;
			int						mY = 0;
			std::vector<int32_t>	mTriangles;		///< Vertex indices, 3 per triangle
			unsigned char *			mData = nullptr;	///< dtCreateNavMeshData output (dtAlloc)
			int						mDataSize = 0;
		};

		const float *				mVertices = nullptr;
		uint32_t					mVertexCount = 0;
		sn_nav_config				mConfig {};
		float						mBoundsMin[3] {};
		float						mBoundsMax[3] {};
		int							mTilesX = 0;
		int							mTilesY = 0;
		std::vector<Tile>			mTiles;
	};

	void DefaultNavConfig(sn_nav_config &outConfig);

	/// Serialized form used by sn_nav_load / st_nav_bake: header, navmesh params, then tiles.
	void SerializeNavMesh(const dtNavMesh &inNavMesh, std::vector<uint8_t> &outData);
	dtNavMesh *DeserializeNavMesh(const void *inData, uint32_t inSize, std::string &outError);
}
