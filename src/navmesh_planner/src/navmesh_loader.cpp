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

NavMeshLoadResult loadNavMeshAuto(const std::string& path,
                                  dtTileCacheAlloc* alloc,
                                  dtTileCacheCompressor* comp,
                                  dtTileCacheMeshProcess* meshProc) {
  NavMeshLoadResult result = {nullptr, nullptr};

  FILE* file = fopen(path.c_str(), "rb");
  if (!file) {
    return result;
  }

  // Read magic bytes to detect format
  int magic = 0;
  size_t readLen = fread(&magic, sizeof(int), 1, file);
  if (readLen != 1) {
    fclose(file);
    return result;
  }
  fseek(file, 0, SEEK_SET);  // Rewind to start

  if (magic == NAVMESHSET_MAGIC) {
    // MSET format — delegate to existing loader
    fclose(file);
    result.navMesh = loadNavMesh(path);
    return result;
  }

  if (magic != TILECACHESET_MAGIC) {
    // Unknown format
    fclose(file);
    return result;
  }

  // ---- TSET format: load TileCache + NavMesh ----

  TileCacheSetHeader header;
  readLen = fread(&header, sizeof(TileCacheSetHeader), 1, file);
  if (readLen != 1) {
    fclose(file);
    return result;
  }
  if (header.version != TILECACHESET_VERSION) {
    fclose(file);
    return result;
  }

  // Init NavMesh
  dtNavMesh* mesh = dtAllocNavMesh();
  if (!mesh) {
    fclose(file);
    return result;
  }
  dtStatus status = mesh->init(&header.meshParams);
  if (dtStatusFailed(status)) {
    dtFreeNavMesh(mesh);
    fclose(file);
    return result;
  }

  // Init TileCache
  dtTileCache* tileCache = dtAllocTileCache();
  if (!tileCache) {
    dtFreeNavMesh(mesh);
    fclose(file);
    return result;
  }
  status = tileCache->init(&header.cacheParams, alloc, comp, meshProc);
  if (dtStatusFailed(status)) {
    dtFreeTileCache(tileCache);
    dtFreeNavMesh(mesh);
    fclose(file);
    return result;
  }

  // Read compressed tiles
  for (int i = 0; i < header.numTiles; ++i) {
    TileCacheTileHeader tileHeader;
    readLen = fread(&tileHeader, sizeof(tileHeader), 1, file);
    if (readLen != 1) break;
    if (!tileHeader.tileRef || !tileHeader.dataSize) break;

    unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
    if (!data) break;
    memset(data, 0, tileHeader.dataSize);
    readLen = fread(data, tileHeader.dataSize, 1, file);
    if (readLen != 1) {
      dtFree(data);
      break;
    }

    dtCompressedTileRef tileRef = 0;
    tileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tileRef);
    if (tileRef) {
      tileCache->buildNavMeshTile(tileRef, mesh);
    }
  }

  fclose(file);
  result.navMesh = mesh;
  result.tileCache = tileCache;
  return result;
}
