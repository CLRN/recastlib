#pragma once

#include "DetourAlloc.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

#include <cstddef>
#include <cstdint>
#include <unordered_map>

class Unit;

//  memory management
inline void* dtCustomAlloc(size_t size, dtAllocHint /*hint*/)
{
    return (void*)new unsigned char[size];
}

inline void dtCustomFree(void* ptr)
{
    delete[](unsigned char*)ptr;
}

//  move map related classes
namespace MMAP
{
    typedef std::unordered_map<uint32_t, dtTileRef> MMapTileSet;
    typedef std::unordered_map<uint32_t, dtNavMeshQuery*> NavMeshQuerySet;

    // dummy struct to hold map's mmap data
    struct MMapData
    {
        MMapData(dtNavMesh* mesh) : navMesh(mesh) {}
        ~MMapData()
        {
            for (NavMeshQuerySet::iterator i = navMeshQueries.begin(); i != navMeshQueries.end(); ++i)
                { dtFreeNavMeshQuery(i->second); }

            if (navMesh)
                { dtFreeNavMesh(navMesh); }
        }

        dtNavMesh* navMesh;

        // we have to use single dtNavMeshQuery for every instance, since those are not thread safe
        NavMeshQuerySet navMeshQueries;     // instanceId to query
        MMapTileSet mmapLoadedTiles;        // maps [map grid coords] to [dtTile]
    };


    typedef std::unordered_map<uint32_t, MMapData*> MMapDataSet;

    // singelton class
    // holds all all access to mmap loading unloading and meshes
    class MMapManager
    {
        public:
            MMapManager(const std::string& dir) : loadedTiles(0), dataDir(dir) {}
            ~MMapManager();

            bool loadMap(uint32_t mapId, int32_t x, int32_t y);
            bool unloadMap(uint32_t mapId, int32_t x, int32_t y);
            bool unloadMap(uint32_t mapId);
            bool unloadMapInstance(uint32_t mapId, uint32_t instanceId);
            bool loadMapAndAllTiles(uint32_t mapId);

            // the returned [dtNavMeshQuery const*] is NOT threadsafe
            dtNavMeshQuery const* GetNavMeshQuery(uint32_t mapId, uint32_t instanceId = 0);
            dtNavMesh const* GetNavMesh(uint32_t mapId);

            uint32_t getLoadedTilesCount() const { return loadedTiles; }
            uint32_t getLoadedMapsCount() const { return loadedMMaps.size(); }
        private:
            bool loadMapData(uint32_t mapId);
            uint32_t packTileID(int32_t x, int32_t y);

            MMapDataSet loadedMMaps;
            uint32_t loadedTiles;
            const std::string dataDir;
    };

    // static class
    // holds all mmap global data
    // access point to MMapManager singelton
    class MMapFactory
    {
        public:
            static void createManager(const std::string& dir);
            static MMapManager& manager();
            static void clear();
    };
}
