#include "mmap.h"
#include "mmapdefines.h"
#include "DetourAssert.h"

#include <set>
#include <cstring>

namespace MMAP
{
    std::string GetDataPath()
    {
        return ".";
    }

    template<typename... Args>
    void outError(const char* text, Args... args)
    {
        std::fprintf(stderr, text, args...);
    }

    template<typename... Args>
    void outInfo(const char* text, Args... args)
    {
        std::fprintf(stderr, text, args...);
    }

    // ######################## MMapFactory ########################
    // our global singelton copy
    MMapManager* g_MMapManager = NULL;

    // stores list of mapids which do not use pathfinding
    std::set<uint32_t>* g_mmapDisabledIds = NULL;

    MMapManager* MMapFactory::createOrGetMMapManager()
    {
        if (g_MMapManager == NULL)
            { g_MMapManager = new MMapManager(); }

        return g_MMapManager;
    }

    void MMapFactory::preventPathfindingOnMaps(const char* ignoreMapIds)
    {
        if (!g_mmapDisabledIds)
            { g_mmapDisabledIds = new std::set<uint32_t>(); }

        uint32_t strLenght = strlen(ignoreMapIds) + 1;
        char* mapList = new char[strLenght];
        memcpy(mapList, ignoreMapIds, sizeof(char)*strLenght);

        char* idstr = strtok(mapList, ",");
        while (idstr)
        {
            g_mmapDisabledIds->insert(uint32_t(atoi(idstr)));
            idstr = strtok(NULL, ",");
        }

        delete[] mapList;
    }

    void MMapFactory::clear()
    {
        delete g_mmapDisabledIds;
        delete g_MMapManager;

        g_mmapDisabledIds = NULL;
        g_MMapManager = NULL;
    }

    // ######################## MMapManager ########################
    MMapManager::~MMapManager()
    {
        for (MMapDataSet::iterator i = loadedMMaps.begin(); i != loadedMMaps.end(); ++i)
            { delete i->second; }

        // by now we should not have maps loaded
        // if we had, tiles in MMapData->mmapLoadedTiles, their actual data is lost!
    }

    bool MMapManager::loadMapData(uint32_t mapId)
    {
        // we already have this map loaded?
        if (loadedMMaps.find(mapId) != loadedMMaps.end())
            { return true; }

        // load and init dtNavMesh - read parameters from file
        char fileName[4096] = {};
        snprintf(fileName, sizeof(fileName), (GetDataPath() + "mmaps/%03i.mmap").c_str(), mapId);

        FILE* file = fopen(fileName, "rb");
        if (!file)
        {
            outError("MMAP:loadMapData: Error: Could not open mmap file '%s'", fileName);
            return false;
        }

        dtNavMeshParams params;
        size_t file_read = fread(&params, sizeof(dtNavMeshParams), 1, file);
        if (file_read <= 0)
        {
            outError("MMAP:loadMapData: Failed to load mmap %03u from file %s", mapId, fileName);
            fclose(file);
            return false;
        }
        fclose(file);

        dtNavMesh* mesh = dtAllocNavMesh();
        dtAssert(mesh);
        dtStatus dtResult = mesh->init(&params);
        if (dtStatusFailed(dtResult))
        {
            dtFreeNavMesh(mesh);
            outError("MMAP:loadMapData: Failed to initialize dtNavMesh for mmap %03u from file %s", mapId, fileName);
            return false;
        }

        outInfo("MMAP:loadMapData: Loaded %03i.mmap", mapId);

        // store inside our map list
        MMapData* mmap_data = new MMapData(mesh);
        mmap_data->mmapLoadedTiles.clear();

        loadedMMaps.insert(std::pair<uint32_t, MMapData*>(mapId, mmap_data));
        return true;
    }

    uint32_t MMapManager::packTileID(int32_t x, int32_t y)
    {
        return uint32_t(x << 16 | y);
    }

    bool MMapManager::loadMap(uint32_t mapId, int32_t x, int32_t y)
    {
        // make sure the mmap is loaded and ready to load tiles
        if (!loadMapData(mapId))
            { return false; }

        // get this mmap data
        MMapData* mmap = loadedMMaps[mapId];
        dtAssert(mmap->navMesh);

        // check if we already have this tile loaded
        uint32_t packedGridPos = packTileID(x, y);
        if (mmap->mmapLoadedTiles.find(packedGridPos) != mmap->mmapLoadedTiles.end())
        {
            outError("MMAP:loadMap: Asked to load already loaded navmesh tile. %03u%02i%02i.mmtile", mapId, x, y);
            return false;
        }

        // load this tile :: mmaps/MMMXXYY.mmtile
        char fileName[4096] = {};
        snprintf(fileName, sizeof(fileName), (GetDataPath() + "mmaps/%03i%02i%02i.mmtile").c_str(), mapId, x, y);

        FILE* file = fopen(fileName, "rb");
        if (!file)
        {
            outError("ERROR: MMAP:loadMap: Could not open mmtile file '%s'", fileName);
            return false;
        }

        // read header
        MmapTileHeader fileHeader;
        size_t file_read = fread(&fileHeader, sizeof(MmapTileHeader), 1, file);

        if (file_read <= 0)
        {
            outError("MMAP:loadMap: Could not load mmap %03u%02i%02i.mmtile", mapId, x, y);
            fclose(file);
            return false;
        }

        if (fileHeader.mmapMagic != MMAP_MAGIC)
        {
            outError("MMAP:loadMap: Bad header in mmap %03u%02i%02i.mmtile", mapId, x, y);
            fclose(file);
            return false;
        }

        if (fileHeader.mmapVersion != MMAP_VERSION)
        {
            outError("MMAP:loadMap: %03u%02i%02i.mmtile was built with generator v%i, expected v%i",
                          mapId, x, y, fileHeader.mmapVersion, MMAP_VERSION);
            fclose(file);
            return false;
        }

        unsigned char* data = (unsigned char*)dtAlloc(fileHeader.size, DT_ALLOC_PERM);
        dtAssert(data);

        size_t result = fread(data, fileHeader.size, 1, file);
        if (!result)
        {
            outError("MMAP:loadMap: Bad header or data in mmap %03u%02i%02i.mmtile", mapId, x, y);
            fclose(file);
            return false;
        }

        fclose(file);

        dtMeshHeader* header = (dtMeshHeader*)data;
        dtTileRef tileRef = 0;

        // memory allocated for data is now managed by detour, and will be deallocated when the tile is removed
        dtStatus dtResult = mmap->navMesh->addTile(data, fileHeader.size, DT_TILE_FREE_DATA, 0, &tileRef);
        if (dtStatusFailed(dtResult))
        {
            outError("MMAP:loadMap: Could not load %03u%02i%02i.mmtile into navmesh", mapId, x, y);
            dtFree(data);
            return false;
        }

        mmap->mmapLoadedTiles.insert(std::pair<uint32_t, dtTileRef>(packedGridPos, tileRef));
        ++loadedTiles;
        outInfo("MMAP:loadMap: Loaded mmtile %03i[%02i,%02i] into %03i[%02i,%02i]", mapId, x, y, mapId, header->x, header->y);
        return true;
    }

    bool MMapManager::unloadMap(uint32_t mapId, int32_t x, int32_t y)
    {
        // check if we have this map loaded
        if (loadedMMaps.find(mapId) == loadedMMaps.end())
        {
            // file may not exist, therefore not loaded
            outInfo("MMAP:unloadMap: Asked to unload not loaded navmesh map. %03u%02i%02i.mmtile", mapId, x, y);
            return false;
        }

        MMapData* mmap = loadedMMaps[mapId];

        // check if we have this tile loaded
        uint32_t packedGridPos = packTileID(x, y);
        if (mmap->mmapLoadedTiles.find(packedGridPos) == mmap->mmapLoadedTiles.end())
        {
            // file may not exist, therefore not loaded
            outInfo("MMAP:unloadMap: Asked to unload not loaded navmesh tile. %03u%02i%02i.mmtile", mapId, x, y);
            return false;
        }

        dtTileRef tileRef = mmap->mmapLoadedTiles[packedGridPos];

        // unload, and mark as non loaded
        dtStatus dtResult = mmap->navMesh->removeTile(tileRef, NULL, NULL);
        if (dtStatusFailed(dtResult))
        {
            // this is technically a memory leak
            // if the grid is later reloaded, dtNavMesh::addTile will return error but no extra memory is used
            // we can not recover from this error - dtAssert out
            outError("MMAP:unloadMap: Could not unload %03u%02i%02i.mmtile from navmesh", mapId, x, y);
            dtAssert(false);
        }
        else
        {
            mmap->mmapLoadedTiles.erase(packedGridPos);
            --loadedTiles;
            outInfo("MMAP:unloadMap: Unloaded mmtile %03i[%02i,%02i] from %03i", mapId, x, y, mapId);
            return true;
        }

        return false;
    }

    bool MMapManager::unloadMap(uint32_t mapId)
    {
        if (loadedMMaps.find(mapId) == loadedMMaps.end())
        {
            // file may not exist, therefore not loaded
            outInfo("MMAP:unloadMap: Asked to unload not loaded navmesh map %03u", mapId);
            return false;
        }

        // unload all tiles from given map
        MMapData* mmap = loadedMMaps[mapId];
        for (MMapTileSet::iterator i = mmap->mmapLoadedTiles.begin(); i != mmap->mmapLoadedTiles.end(); ++i)
        {
            uint32_t x = (i->first >> 16);
            uint32_t y = (i->first & 0x0000FFFF);
            dtStatus dtResult = mmap->navMesh->removeTile(i->second, NULL, NULL);
            if (dtStatusFailed(dtResult))
                { outError("MMAP:unloadMap: Could not unload %03u%02i%02i.mmtile from navmesh", mapId, x, y); }
            else
            {
                --loadedTiles;
                outInfo("MMAP:unloadMap: Unloaded mmtile %03i[%02i,%02i] from %03i", mapId, x, y, mapId);
            }
        }

        delete mmap;
        loadedMMaps.erase(mapId);
        outInfo("MMAP:unloadMap: Unloaded %03i.mmap", mapId);

        return true;
    }

    bool MMapManager::unloadMapInstance(uint32_t mapId, uint32_t instanceId)
    {
        // check if we have this map loaded
        if (loadedMMaps.find(mapId) == loadedMMaps.end())
        {
            // file may not exist, therefore not loaded
            outInfo("MMAP:unloadMapInstance: Asked to unload not loaded navmesh map %03u", mapId);
            return false;
        }

        MMapData* mmap = loadedMMaps[mapId];
        if (mmap->navMeshQueries.find(instanceId) == mmap->navMeshQueries.end())
        {
            outInfo("MMAP:unloadMapInstance: Asked to unload not loaded dtNavMeshQuery mapId %03u instanceId %u", mapId, instanceId);
            return false;
        }

        dtNavMeshQuery* query = mmap->navMeshQueries[instanceId];

        dtFreeNavMeshQuery(query);
        mmap->navMeshQueries.erase(instanceId);
        outInfo("MMAP:unloadMapInstance: Unloaded mapId %03u instanceId %u", mapId, instanceId);

        return true;
    }

    dtNavMesh const* MMapManager::GetNavMesh(uint32_t mapId)
    {
        if (loadedMMaps.find(mapId) == loadedMMaps.end())
            { return NULL; }

        return loadedMMaps[mapId]->navMesh;
    }

    dtNavMeshQuery const* MMapManager::GetNavMeshQuery(uint32_t mapId, uint32_t instanceId)
    {
        if (loadedMMaps.find(mapId) == loadedMMaps.end())
            { return NULL; }

        MMapData* mmap = loadedMMaps[mapId];
        if (mmap->navMeshQueries.find(instanceId) == mmap->navMeshQueries.end())
        {
            // allocate mesh query
            dtNavMeshQuery* query = dtAllocNavMeshQuery();
            dtAssert(query);
            dtStatus dtResult = query->init(mmap->navMesh, 1024);
            if (dtStatusFailed(dtResult))
            {
                dtFreeNavMeshQuery(query);
                outError("MMAP:GetNavMeshQuery: Failed to initialize dtNavMeshQuery for mapId %03u instanceId %u", mapId, instanceId);
                return NULL;
            }

            outInfo("MMAP:GetNavMeshQuery: created dtNavMeshQuery for mapId %03u instanceId %u", mapId, instanceId);
            mmap->navMeshQueries.insert(std::pair<uint32_t, dtNavMeshQuery*>(instanceId, query));
        }

        return mmap->navMeshQueries[instanceId];
    }
}
