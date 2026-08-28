#ifndef NAVMESH_PLANNER_TILECACHE_HELPERS_HPP
#define NAVMESH_PLANNER_TILECACHE_HELPERS_HPP

#include "DetourTileCacheBuilder.h"
#include "DetourAlloc.h"
#include "DetourCommon.h"
#include "DetourNavMeshBuilder.h"
#include "fastlz.h"

#include <cstring>

// Polygon area and flag constants (simplified — no swim/door/off-mesh)
static const int SAMPLE_POLYAREA_GROUND = 0;
static const unsigned short SAMPLE_POLYFLAGS_WALK = 0x01;

// Bump allocator for temporary tile rebuild memory.
// Allocated once, reused across all tile rebuilds via reset().
struct LinearAllocator : public dtTileCacheAlloc
{
    unsigned char* buffer;
    size_t capacity;
    size_t top;
    size_t high;

    explicit LinearAllocator(const size_t cap)
        : buffer(nullptr), capacity(0), top(0), high(0)
    {
        resize(cap);
    }

    ~LinearAllocator() override
    {
        dtFree(buffer);
    }

    void resize(const size_t cap)
    {
        if (buffer) dtFree(buffer);
        buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
        capacity = cap;
    }

    void reset() override
    {
        high = dtMax(high, top);
        top = 0;
    }

    void* alloc(const size_t size) override
    {
        if (!buffer) return nullptr;
        if (top + size > capacity) return nullptr;
        unsigned char* mem = &buffer[top];
        top += size;
        return mem;
    }

    void free(void* /*ptr*/) override
    {
        // Linear allocator does not free individual allocations
    }
};

// Wraps FastLZ compression used by RecastDemo's TSET format.
struct FastLZCompressor : public dtTileCacheCompressor
{
    int maxCompressedSize(const int bufferSize) override
    {
        return (int)((float)bufferSize * 1.05f);
    }

    dtStatus compress(const unsigned char* buffer, const int bufferSize,
                      unsigned char* compressed, const int /*maxCompressedSize*/,
                      int* compressedSize) override
    {
        *compressedSize = fastlz_compress((const void*)buffer, bufferSize, compressed);
        return DT_SUCCESS;
    }

    dtStatus decompress(const unsigned char* compressed, const int compressedSize,
                        unsigned char* buffer, const int maxBufferSize,
                        int* bufferSize) override
    {
        *bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
        return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
    }
};

// Post-build callback: sets polygon areas and flags during tile rebuild.
// Simplified version — no swim/door areas, no off-mesh connections.
struct MeshProcess : public dtTileCacheMeshProcess
{
    void process(struct dtNavMeshCreateParams* params,
                 unsigned char* polyAreas,
                 unsigned short* polyFlags) override
    {
        for (int i = 0; i < params->polyCount; ++i)
        {
            if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA)
                polyAreas[i] = SAMPLE_POLYAREA_GROUND;

            if (polyAreas[i] == SAMPLE_POLYAREA_GROUND)
                polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
        }
    }
};

#endif // NAVMESH_PLANNER_TILECACHE_HELPERS_HPP
