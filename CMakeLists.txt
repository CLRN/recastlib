cmake_minimum_required (VERSION 3.2)
project (RecastLib)
set (RecastLib_VERSION_MAJOR 0)
set (RecastLib_VERSION_MINOR 2)

find_package(Python 3.7 REQUIRED COMPONENTS Development)
find_package(Boost REQUIRED COMPONENTS python3 filesystem)

include_directories (${Boost_INCLUDE_DIRS})
include_directories (${Python_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/Recast/DebugUtils/Include")
include_directories ("${PROJECT_SOURCE_DIR}/Recast/Detour/Include")
include_directories ("${PROJECT_SOURCE_DIR}/Recast/DetourCrowd/Include")
include_directories ("${PROJECT_SOURCE_DIR}/Recast/DetourTileCache/Include")
include_directories ("${PROJECT_SOURCE_DIR}/Recast/Recast/Include")
include_directories ("${PROJECT_SOURCE_DIR}/python")

add_library (_recast SHARED
    ${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourAlloc.cpp
    ${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourCommon.cpp
    ${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourNavMesh.cpp
    ${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourNavMeshBuilder.cpp
    ${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourNavMeshQuery.cpp
    ${PROJECT_SOURCE_DIR}/Recast/Detour/Source/DetourNode.cpp
    ${PROJECT_SOURCE_DIR}/python/detour/query.cpp
    ${PROJECT_SOURCE_DIR}/python/loader/tilemesh.cpp
    ${PROJECT_SOURCE_DIR}/python/loader/sampletilemesh.cpp
    ${PROJECT_SOURCE_DIR}/python/detour.cpp
    ${PROJECT_SOURCE_DIR}/python/dtmath.cpp
    ${PROJECT_SOURCE_DIR}/python/loader.cpp
    ${PROJECT_SOURCE_DIR}/python/export.cpp
    ${PROJECT_SOURCE_DIR}/python/assert.cpp
    ${PROJECT_SOURCE_DIR}/python/mmap.cpp
    ${PROJECT_SOURCE_DIR}/python/mmap_export.cpp)

target_link_libraries(_recast ${Python_LIBRARY} Boost::python3 Boost::filesystem)

add_executable(test_load test/main.cpp)
target_link_libraries(test_load _recast)
