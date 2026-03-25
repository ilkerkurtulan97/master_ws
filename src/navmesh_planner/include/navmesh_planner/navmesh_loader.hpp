#pragma once

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include <string>

// Matches RecastDemo's binary format exactly
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

// Load a .bin navmesh saved by RecastDemo's Sample::saveAll()
// Returns nullptr on failure. Caller owns the returned mesh (free with dtFreeNavMesh).
dtNavMesh* loadNavMesh(const std::string& path);
