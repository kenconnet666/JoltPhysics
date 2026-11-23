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

		/// Validates the input, computes the tile grid over the input's bounds and bins triangles into
		/// tiles. The input arrays are referenced, not copied.
		bool Prepare(const float *inVertices, uint32_t inVertexCount, const int32_t *inTriangles, uint32_t inTriangleCount, const sn_nav_config &inConfig, std::string &outError);

		/// Like Prepare, but for the given tiles (x, y pairs) of a grid whose tile (0, 0) starts at
		/// inOrigin (the navmesh's orig): rebuilding parts of an existing tiled navmesh. No geometry is
		/// fine: those tiles are removed by ReplaceTiles.
		bool PrepareTiles(const float *inVertices, uint32_t inVertexCount, const int32_t *inTriangles, uint32_t inTriangleCount, const sn_nav_config &inConfig,
			const float inOrigin[3], const int32_t *inTilesXY, uint32_t inTileCount, std::string &outError);

		uint32_t GetTileCount() const { return uint32_t(mTiles.size()); }

		/// Builds one tile; tiles without walkable area produce no data. Thread-safe across tiles.
		bool BuildTile(uint32_t inTileIndex, std::string &outError);

		/// Creates the navmesh from the built tiles, which it then owns. Call once, after all tiles.
		dtNavMesh *CreateNavMesh(std::string &outError);

		/// Replaces the prepared tiles in ioNavMesh with the built ones (a tile without walkable area
		/// is removed). Returns how many tiles have data, or -1 on failure.
		int ReplaceTiles(dtNavMesh &ioNavMesh, std::string &outError);

	private:
		struct Tile
		{
			int						mX = 0;
			int						mY = 0;
			std::vector<int32_t>	mTriangles;		///< Vertex indices, 3 per triangle
			unsigned char *			mData = nullptr;	///< dtCreateNavMeshData output (dtAlloc)
			int						mDataSize = 0;
		};

		bool Validate(const float *inVertices, uint32_t inVertexCount, const int32_t *inTriangles, uint32_t inTriangleCount, const sn_nav_config &inConfig, std::string &outError);
		/// Bins each triangle into every prepared tile its xz bounds touch, including the tile border.
		void BinTriangles(const int32_t *inTriangles, uint32_t inTriangleCount);

		const float *				mVertices = nullptr;
		uint32_t					mVertexCount = 0;
		sn_nav_config				mConfig {};
		float						mOrigin[3] {};		///< Corner of tile (0, 0); y unused
		float						mBoundsMin[3] {};	///< Input bounds; the y range is the heightfield's
		float						mBoundsMax[3] {};
		int							mTilesX = 0;
		int							mTilesY = 0;
		std::vector<Tile>			mTiles;
	};

	void DefaultNavConfig(sn_nav_config &outConfig);

	/// An empty tiled navmesh with orig (0, 0, 0), filled later with NavMeshBuilder::PrepareTiles and ReplaceTiles.
	dtNavMesh *CreateTiledNavMesh(const sn_nav_config &inConfig, uint32_t inMaxTiles, std::string &outError);

	/// Serialized form used by sn_nav_load / st_nav_bake: header, navmesh params, then tiles.
	void SerializeNavMesh(const dtNavMesh &inNavMesh, std::vector<uint8_t> &outData);
	dtNavMesh *DeserializeNavMesh(const void *inData, uint32_t inSize, std::string &outError);
}
