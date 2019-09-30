#include "mmap.h"
#include "Config.h"
#include "mmapdefines.h"
#include <cassert>

#include <boost/format.hpp>

namespace MMAP
{
    static char const* const MAP_FILE_NAME_FORMAT = "%s/mmaps/%03i.mmap";
    static char const* const TILE_FILE_NAME_FORMAT = "%s/mmaps/%03i%02i%02i.mmtile";

    template<typename Format, typename... Args>
    inline std::string StringFormat(Format&& fmt, Args&&... args)
    {
        try
        {
            boost::format f(fmt);
            int unroll[] {0, (f % std::forward<Args>(args), 0)...};
            static_cast<void>(unroll);
            return f.str();
        }
        catch (const std::exception& formatError)
        {
            std::string error = "An error occurred formatting string \"" + std::string(fmt) + "\" : " + std::string(formatError.what());
            return error;
        }
    }

    template<typename... Args>
    void outError(const char* text, Args... args)
    {
        char buffer[4096] = {};
        std::sprintf(buffer, text, args...);
        std::cerr << buffer << std::endl;
    }

    template<typename... Args>
    void outInfo(const char* text, Args... args)
    {
        char buffer[4096] = {};
        std::sprintf(buffer, text, args...);
        std::cerr << buffer << std::endl;
    }

    // ######################## MMapFactory ########################
    // our global singelton copy
    MMapManager* g_MMapManager = NULL;

    void MMapFactory::clear()
    {
        delete g_MMapManager;

        g_MMapManager = NULL;
    }

    void MMapFactory::createManager(const std::string &dir) {
        g_MMapManager = new MMapManager(dir);
    }

    MMapManager &MMapFactory::manager() {
        return *g_MMapManager;;
    }

    // ######################## MMapManager ########################
    MMapManager::~MMapManager()
    {
        for (MMapDataSet::iterator i = loadedMMaps.begin(); i != loadedMMaps.end(); ++i)
            delete i->second;

        // by now we should not have maps loaded
        // if we had, tiles in MMapData->mmapLoadedTiles, their actual data is lost!
    }

    void MMapManager::InitializeThreadUnsafe(const std::vector<uint32_t>& mapIds)
    {
        // the caller must pass the list of all mapIds that will be used in the VMapManager2 lifetime
        for (uint32_t const& mapId : mapIds)
            loadedMMaps.insert(MMapDataSet::value_type(mapId, nullptr));

        thread_safe_environment = false;
    }

    MMapDataSet::const_iterator MMapManager::GetMMapData(uint32_t mapId) const
    {
        // return the iterator if found or end() if not found/NULL
        MMapDataSet::const_iterator itr = loadedMMaps.find(mapId);
        if (itr != loadedMMaps.cend() && !itr->second)
            itr = loadedMMaps.cend();

        return itr;
    }

    bool MMapManager::loadMapData(uint32_t mapId)
    {
        // we already have this map loaded?
        MMapDataSet::iterator itr = loadedMMaps.find(mapId);
        if (itr != loadedMMaps.end())
        {
            if (itr->second)
                return true;
        }
        else
        {
            if (thread_safe_environment)
                itr = loadedMMaps.insert(MMapDataSet::value_type(mapId, nullptr)).first;
            else
                outError("Invalid mapId %u passed to MMapManager after startup in thread unsafe environment", mapId);
        }

        // load and init dtNavMesh - read parameters from file
        std::string fileName = StringFormat(MAP_FILE_NAME_FORMAT, dataDir, mapId);
        FILE* file = fopen(fileName.c_str(), "rb");
        if (!file)
        {
            outInfo("MMAP:loadMapData: Error: Could not open mmap file '%s'", fileName.c_str());
            return false;
        }

        dtNavMeshParams params;
        uint32_t count = uint32_t(fread(&params, sizeof(dtNavMeshParams), 1, file));
        fclose(file);
        if (count != 1)
        {
            outInfo("MMAP:loadMapData: Error: Could not read params from file '%s'", fileName.c_str());
            return false;
        }

        dtNavMesh* mesh = dtAllocNavMesh();
        assert(mesh);
        if (dtStatusFailed(mesh->init(&params)))
        {
            dtFreeNavMesh(mesh);
            outError("MMAP:loadMapData: Failed to initialize dtNavMesh for mmap %03u from file %s", mapId, fileName.c_str());
            return false;
        }

        outInfo("MMAP:loadMapData: Loaded %03i.mmap", mapId);

        // store inside our map list
        MMapData* mmap_data = new MMapData(mesh);

        itr->second = mmap_data;
        return true;
    }

    uint32_t MMapManager::packTileID(int32_t x, int32_t y)
    {
        return uint32_t(x << 16 | y);
    }

    bool MMapManager::loadMap(uint32_t mapId, uint32_t x, uint32_t y)
    {
        // make sure the mmap is loaded and ready to load tiles
        if (!loadMapData(mapId))
            return false;

        // get this mmap data
        MMapData* mmap = loadedMMaps[mapId];
        assert(mmap->navMesh);

        // check if we already have this tile loaded
        uint32_t packedGridPos = packTileID(x, y);
        if (mmap->loadedTileRefs.find(packedGridPos) != mmap->loadedTileRefs.end())
            return false;

        // load this tile :: mmaps/MMMXXYY.mmtile
        std::string fileName = StringFormat(TILE_FILE_NAME_FORMAT, dataDir, mapId, x, y);
        FILE* file = fopen(fileName.c_str(), "rb");
        if (!file)
        {
            outInfo("MMAP:loadMap: Could not open mmtile file '%s'", fileName.c_str());
            return false;
        }

        // read header
        MmapTileHeader fileHeader;
        if (fread(&fileHeader, sizeof(MmapTileHeader), 1, file) != 1 || fileHeader.mmapMagic != MMAP_MAGIC)
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

        long pos = ftell(file);
        fseek(file, 0, SEEK_END);
        if (pos < 0 || static_cast<int32_t>(fileHeader.size) > ftell(file) - pos)
        {
            outError("MMAP:loadMap: %03u%02i%02i.mmtile has corrupted data size", mapId, x, y);
            fclose(file);
            return false;
        }

        fseek(file, pos, SEEK_SET);

        unsigned char* data = (unsigned char*)dtAlloc(fileHeader.size, DT_ALLOC_PERM);
        assert(data);

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
        if (dtStatusSucceed(mmap->navMesh->addTile(data, fileHeader.size, DT_TILE_FREE_DATA, 0, &tileRef)))
        {
            mmap->loadedTileRefs.insert(std::pair<uint32_t, dtTileRef>(packedGridPos, tileRef));
            ++loadedTiles;
            outInfo("MMAP:loadMap: Loaded mmtile %03i[%02i, %02i] into %03i[%02i, %02i]", mapId, x, y, mapId, header->x, header->y);
            return true;
        }
        else
        {
            outError("MMAP:loadMap: Could not load %03u%02i%02i.mmtile into navmesh", mapId, x, y);
            dtFree(data);
            return false;
        }
    }

    bool MMapManager::unloadMap(uint32_t mapId, int32_t x, int32_t y)
    {
        // check if we have this map loaded
        MMapDataSet::const_iterator itr = GetMMapData(mapId);
        if (itr == loadedMMaps.end())
        {
            // file may not exist, therefore not loaded
            outInfo("MMAP:unloadMap: Asked to unload not loaded navmesh map. %03u%02i%02i.mmtile", mapId, x, y);
            return false;
        }

        MMapData* mmap = itr->second;

        // check if we have this tile loaded
        uint32_t packedGridPos = packTileID(x, y);
        if (mmap->loadedTileRefs.find(packedGridPos) == mmap->loadedTileRefs.end())
        {
            // file may not exist, therefore not loaded
            outInfo("MMAP:unloadMap: Asked to unload not loaded navmesh tile. %03u%02i%02i.mmtile", mapId, x, y);
            return false;
        }

        dtTileRef tileRef = mmap->loadedTileRefs[packedGridPos];

        // unload, and mark as non loaded
        if (dtStatusFailed(mmap->navMesh->removeTile(tileRef, nullptr, nullptr)))
        {
            // this is technically a memory leak
            // if the grid is later reloaded, dtNavMesh::addTile will return error but no extra memory is used
            // we cannot recover from this error - assert out
            outError("MMAP:unloadMap: Could not unload %03u%02i%02i.mmtile from navmesh", mapId, x, y);
            std::abort();
        }
        else
        {
            mmap->loadedTileRefs.erase(packedGridPos);
            --loadedTiles;
            outInfo("MMAP:unloadMap: Unloaded mmtile %03i[%02i, %02i] from %03i", mapId, x, y, mapId);
            return true;
        }

        return false;
    }

    bool MMapManager::unloadMap(uint32_t mapId)
    {
        MMapDataSet::iterator itr = loadedMMaps.find(mapId);
        if (itr == loadedMMaps.end() || !itr->second)
        {
            // file may not exist, therefore not loaded
            outInfo("MMAP:unloadMap: Asked to unload not loaded navmesh map %03u", mapId);
            return false;
        }

        // unload all tiles from given map
        MMapData* mmap = itr->second;
        for (MMapTileSet::iterator i = mmap->loadedTileRefs.begin(); i != mmap->loadedTileRefs.end(); ++i)
        {
            uint32_t x = (i->first >> 16);
            uint32_t y = (i->first & 0x0000FFFF);
            if (dtStatusFailed(mmap->navMesh->removeTile(i->second, nullptr, nullptr)))
                outError("MMAP:unloadMap: Could not unload %03u%02i%02i.mmtile from navmesh", mapId, x, y);
            else
            {
                --loadedTiles;
                outInfo("MMAP:unloadMap: Unloaded mmtile %03i[%02i, %02i] from %03i", mapId, x, y, mapId);
            }
        }

        delete mmap;
        itr->second = nullptr;
        outInfo("MMAP:unloadMap: Unloaded %03i.mmap", mapId);

        return true;
    }

    bool MMapManager::unloadMapInstance(uint32_t mapId, uint32_t instanceId)
    {
        // check if we have this map loaded
        MMapDataSet::const_iterator itr = GetMMapData(mapId);
        if (itr == loadedMMaps.end())
        {
            // file may not exist, therefore not loaded
            outInfo("MMAP:unloadMapInstance: Asked to unload not loaded navmesh map %03u", mapId);
            return false;
        }

        MMapData* mmap = itr->second;
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
        MMapDataSet::const_iterator itr = GetMMapData(mapId);
        if (itr == loadedMMaps.end())
            return nullptr;

        return itr->second->navMesh;
    }

    dtNavMeshQuery const* MMapManager::GetNavMeshQuery(uint32_t mapId, uint32_t instanceId)
    {
        MMapDataSet::const_iterator itr = GetMMapData(mapId);
        if (itr == loadedMMaps.end())
            return nullptr;

        MMapData* mmap = itr->second;
        if (mmap->navMeshQueries.find(instanceId) == mmap->navMeshQueries.end())
        {
            // allocate mesh query
            dtNavMeshQuery* query = dtAllocNavMeshQuery();
            assert(query);
            if (dtStatusFailed(query->init(mmap->navMesh, 1024)))
            {
                dtFreeNavMeshQuery(query);
                outError("MMAP:GetNavMeshQuery: Failed to initialize dtNavMeshQuery for mapId %03u instanceId %u", mapId, instanceId);
                return nullptr;
            }

            outInfo("MMAP:GetNavMeshQuery: created dtNavMeshQuery for mapId %03u instanceId %u", mapId, instanceId);
            mmap->navMeshQueries.insert(std::pair<uint32_t, dtNavMeshQuery*>(instanceId, query));
        }

        return mmap->navMeshQueries[instanceId];
    }
}
