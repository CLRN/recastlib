#pragma once

#include "DetourNavMeshQuery.h"

#define MAX_PATH_LENGTH 1024
#define MAX_POINT_PATH_LENGTH   1024
#define VERTEX_SIZE       3

dtStatus FindSmoothPath(dtNavMeshQuery *navMeshQuery,
                        const dtNavMesh *navMesh,
                        float const *startPos, float const *endPos,
                        dtPolyRef const *polyPath, uint32_t polyPathSize,
                        float *smoothPath, int *smoothPathSize, uint32_t maxSmoothPathSize);