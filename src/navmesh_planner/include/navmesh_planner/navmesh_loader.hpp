#pragma once

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include <string>

// ---- MSET format (Solo Mesh / static navmesh) ----
static constexpr int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';
static constexpr int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader {
  int magic;
  int version;
  int numTiles;
  dtNavMeshParams params;
};

struct NavMeshTileHeader {
  dtTileRef tileRef;
  int dataSize;
};

// ---- TSET format (Temp Obstacles / tiled navmesh with dtTileCache) ----
static constexpr int TILECACHESET_MAGIC = 'T' << 24 | 'S' << 16 | 'E' << 8 | 'T';
static constexpr int TILECACHESET_VERSION = 1;

struct TileCacheSetHeader {
  int magic;
  int version;
  int numTiles;
  dtNavMeshParams meshParams;
  dtTileCacheParams cacheParams;
};

struct TileCacheTileHeader {
  dtCompressedTileRef tileRef;
  int dataSize;
};

// Result of loading a navmesh — tileCache is nullptr for MSET (static) meshes.
struct NavMeshLoadResult {
  dtNavMesh* navMesh;
  dtTileCache* tileCache;
};

// Load a .bin navmesh saved by RecastDemo's Sample::saveAll() (MSET format).
// Returns nullptr on failure. Caller owns the returned mesh (free with dtFreeNavMesh).
dtNavMesh* loadNavMesh(const std::string& path);

// Auto-detect MSET vs TSET format and load accordingly.
// For TSET, initializes dtTileCache with the provided helper objects.
// Returns {nullptr, nullptr} on failure.
NavMeshLoadResult loadNavMeshAuto(const std::string& path,
                                  dtTileCacheAlloc* alloc,
                                  dtTileCacheCompressor* comp,
                                  dtTileCacheMeshProcess* meshProc);
