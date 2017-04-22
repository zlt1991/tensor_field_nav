#ifndef _CALPATHINFOGAIN_H
#define _CALPATHINFOGAIN_H
#include <cuda.h>
#include <cuda_runtime.h>
#include "device_launch_parameters.h"
#include <stdio.h>
#include <cmath>

__global__ void calPathInfoGain_device(const float *pointCollection,const float *pathPoints,int *pathInfoGain_tmp,int *pathInfoGain, int pointCollectionSize,int pathPointSize);
extern "C"
{
void calPathInfoGain(const float *pointCollection, const float *pathPoints, int *pathInfoGain,int pointCollectionSize, int pathPointsSize);
}
#endif
