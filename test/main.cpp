#include "mmap.h"

#include "DetourNavMesh.h"

#include <vector>

#define SIZE_OF_GRIDS            533.33333f
#define VERTEX_SIZE 3
#define INVALID_POLYREF 0

int main()
{
    MMAP::MMapFactory::createManager("/mnt/f/tmp/wow_3.3.5/");
    auto& manager = *MMAP::MMapFactory::manager();

    float x = 1555.043212890625, y = -4421.5478515625, z = 8.577301979064941;

    int32_t gx = 32 - x / SIZE_OF_GRIDS;
    int32_t gy = 32 - y / SIZE_OF_GRIDS;

    if (!manager.loadMap(1, gx, gy))
        return 1;

    // calculate navmesh tile location
    dtNavMesh const* navmesh = manager.GetNavMesh(1);
    dtNavMeshQuery const* navmeshquery = manager.GetNavMeshQuery(1);
    if (!navmesh || !navmeshquery)
    {
        return 1;
    }

    float const* min = navmesh->getParams()->orig;
    float startCoords[VERTEX_SIZE] = {y, z, x};
    float extents[VERTEX_SIZE] = {3.0f, 5.0f, 3.0f};

    // navmesh poly -> navmesh tile location
    dtQueryFilter filter = dtQueryFilter();

    dtPolyRef startPoly = INVALID_POLYREF;
    float nearestStart[VERTEX_SIZE] = {};
    if (dtStatusFailed(navmeshquery->findNearestPoly(startCoords, extents, &filter, &startPoly, nearestStart)))
        return 1;

    float ex =  1670.45361328125, ey = -4413.26025390625, ez = 18.072647094726562;
    float endCoords[VERTEX_SIZE] = {ey, ez, ex};

    int32_t egx = 32 - ex / SIZE_OF_GRIDS;
    int32_t egy = 32 - ey / SIZE_OF_GRIDS;

    if (!manager.loadMap(1, egx, egy))
        return 1;

    dtPolyRef endPoly = INVALID_POLYREF;
    float nearestEnd[VERTEX_SIZE] = {};
    if (dtStatusFailed(navmeshquery->findNearestPoly(endCoords, extents, &filter, &endPoly, nearestEnd)))
        return 1;

    static const size_t maxPathLength = 1024;

    dtPolyRef _pathPolyRefs[maxPathLength];
    int pathLength = 0;
    const auto dtResult = navmeshquery->findPath(
            startPoly,          // start polygon
            endPoly,            // end polygon
            startCoords,         // start position
            endCoords,           // end position
            &filter,           // polygon search filter
            _pathPolyRefs,     // [out] path
            &pathLength,
            maxPathLength);   // max number of polygons in output path


    return 0;
}
