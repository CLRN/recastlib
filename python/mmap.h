#pragma once

#include "mmapdefines.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include <string>
#include <unordered_map>
#include <vector>

//  move map related classes
namespace MMAP
{
    typedef std::unordered_map<uint32_t, dtTileRef> MMapTileSet;
    typedef std::unordered_map<uint32_t, dtNavMeshQuery*> NavMeshQuerySet;

    // dummy struct to hold map's mmap data
    struct MMapData
    {
        MMapData(dtNavMesh* mesh) : navMesh(mesh) { }
        ~MMapData()
        {
            for (NavMeshQuerySet::iterator i = navMeshQueries.begin(); i != navMeshQueries.end(); ++i)
                dtFreeNavMeshQuery(i->second);

            if (navMesh)
                dtFreeNavMesh(navMesh);
        }

        // we have to use single dtNavMeshQuery for every instance, since those are not thread safe
        NavMeshQuerySet navMeshQueries;     // instanceId to query

        dtNavMesh* navMesh;
        MMapTileSet loadedTileRefs;        // maps [map grid coords] to [dtTile]
    };


    typedef std::unordered_map<uint32_t, MMapData*> MMapDataSet;

    // singleton class
    // holds all all access to mmap loading unloading and meshes
    class MMapManager
    {
        public:
            MMapManager(const std::string& dir) : loadedTiles(0), thread_safe_environment(true), dataDir(dir) {}
            ~MMapManager();

            void InitializeThreadUnsafe(const std::vector<uint32_t>& mapIds);
            bool loadMap(uint32_t mapId, uint32_t x, uint32_t y);
            bool unloadMap(uint32_t mapId, int32_t x, int32_t y);
            bool unloadMapTiles(uint32_t mapId);
            bool unloadMapInstance(uint32_t mapId, uint32_t instanceId);

            // the returned [dtNavMeshQuery const*] is NOT threadsafe
            dtNavMeshQuery const* GetNavMeshQuery(uint32_t mapId);
            dtNavMesh const* GetNavMesh(uint32_t mapId);

            uint32_t getLoadedTilesCount() const { return loadedTiles; }
            uint32_t getLoadedMapsCount() const { return uint32_t(loadedMMaps.size()); }
        private:
            bool loadMapData(uint32_t mapId);
            uint32_t packTileID(int32_t x, int32_t y);

            MMapDataSet::const_iterator GetMMapData(uint32_t mapId) const;
            MMapDataSet loadedMMaps;
            uint32_t loadedTiles;
            bool thread_safe_environment;
            const std::string dataDir;
    };
}
