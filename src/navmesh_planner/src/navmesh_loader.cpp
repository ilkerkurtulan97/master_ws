#include "navmesh_planner/navmesh_loader.hpp"
#include "DetourAlloc.h"
#include <cstdio>
#include <cstring>

dtNavMesh* loadNavMesh(const std::string& path) {
  FILE* file = fopen(path.c_str(), "rb");
  if (!file) {
    return nullptr;
  }

  // Read header
  NavMeshSetHeader header;
  size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, file);
  if (readLen != 1) {
    fclose(file);
    return nullptr;
  }
  if (header.magic != NAVMESHSET_MAGIC) {
    fclose(file);
    return nullptr;
  }
  if (header.version != NAVMESHSET_VERSION) {
    fclose(file);
    return nullptr;
  }

  dtNavMesh* mesh = dtAllocNavMesh();
  if (!mesh) {
    fclose(file);
    return nullptr;
  }

  dtStatus status = mesh->init(&header.params);
  if (dtStatusFailed(status)) {
    dtFreeNavMesh(mesh);
    fclose(file);
    return nullptr;
  }

  // Read tiles
  for (int i = 0; i < header.numTiles; ++i) {
    NavMeshTileHeader tileHeader;
    readLen = fread(&tileHeader, sizeof(tileHeader), 1, file);
    if (readLen != 1) {
      dtFreeNavMesh(mesh);
      fclose(file);
      return nullptr;
    }

    if (!tileHeader.tileRef || !tileHeader.dataSize) {
      break;
    }

    unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
    if (!data) {
      break;
    }
    memset(data, 0, tileHeader.dataSize);
    readLen = fread(data, tileHeader.dataSize, 1, file);
    if (readLen != 1) {
      dtFree(data);
      dtFreeNavMesh(mesh);
      fclose(file);
      return nullptr;
    }

    mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
  }

  fclose(file);
  return mesh;
}
