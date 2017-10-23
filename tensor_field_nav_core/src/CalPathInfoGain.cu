/***
 Simple implementation of information gains for specified topological branch
 ***/
extern "C" {
#include "tensor_field_nav_core/CalPathInfoGain.h"
}
__global__ void calPathInfoGain_device(const float *pointCollection,const float *pathPoints,int *pathInfoGain_tmp,int *pathInfoGain, int pointCollectionSize,int pathPointSize){
    int idx=blockIdx.x*blockDim.x+threadIdx.x;
    if(idx< pathPointSize){
        int curPointInfoGain=0;
        for(int i=0; i<pointCollectionSize;i++){
            float deta_x=pathPoints[2*idx]-pointCollection[3*i];
            float deta_y=pathPoints[2*idx+1]-pointCollection[3*i+1];
            float deta_z=0.3-pointCollection[3*i+2];
            float dist_quad=(deta_x*deta_x+deta_y*deta_y+deta_z*deta_z);
            if(dist_quad<1)
                curPointInfoGain++;
        }
        pathInfoGain_tmp[idx]=curPointInfoGain;
    }
    __syncthreads();
    if(idx<3){
        pathInfoGain[idx]=pathInfoGain_tmp[idx];
    }else if(idx>pathPointSize-3 && idx<pathPointSize){
        pathInfoGain[idx]=pathInfoGain_tmp[idx];
    }else if(idx>=3 &&idx<=pathPointSize-3){
        int sum_infoGain=0;
        for(int i=idx-2;i<idx+3;i++){
            sum_infoGain=sum_infoGain+pathInfoGain_tmp[idx];
        }
        pathInfoGain[idx]=int(sum_infoGain/5);
    }
}

//extern "C"
void calPathInfoGain(const float *pointCollection, const float *pathPoints, int *pathInfoGain,int pointCollectionSize, int pathPointsSize){
    float *pointCollection_dev,*pathPoints_dev;
    int *pathInfoGain_dev,*pathInfoGain_tmp_dev;
    cudaError_t cudaStatus= cudaMalloc((void**)&pointCollection_dev, sizeof(float)*pointCollectionSize*3);
//    cudaMalloc((void**)&pointCollection_dev, sizeof(float)*pointCollectionSize*3);
    if(cudaStatus !=cudaSuccess){
        fprintf(stderr, "memory malloc to pointCollection failed ");
        return;
    }
    cudaStatus=cudaMalloc((void**)&pathPoints_dev,sizeof(float)*pathPointsSize*2);
    if(cudaStatus !=cudaSuccess){
        fprintf(stderr, "memory malloc to pathPoints failed");
        return;
    }

    cudaStatus=cudaMalloc((void**)&pathInfoGain_dev,sizeof(int)*pathPointsSize);
    if(cudaStatus !=cudaSuccess){
        fprintf(stderr, "memory malloc to pathInfoGain_dev failed");
        return;
    }

    cudaStatus=cudaMalloc((void**)&pathInfoGain_tmp_dev,sizeof(int)*pathPointsSize);
    if(cudaStatus !=cudaSuccess){
        fprintf(stderr, "memory malloc to pathInfoGain_dev failed");
        return;
    }

    cudaStatus=cudaMemcpy(pointCollection_dev,pointCollection,sizeof(float)*pointCollectionSize*3,cudaMemcpyHostToDevice);
    if(cudaStatus !=cudaSuccess){
        fprintf(stderr, "pointCollection memory host to device failed");
        return;
    }

    cudaStatus=cudaMemcpy(pathPoints_dev,pathPoints, sizeof(float)*pathPointsSize*2,cudaMemcpyHostToDevice);
    if(cudaStatus !=cudaSuccess){
        fprintf(stderr, "pathPoints memory host to device failed");
        return;
    }

    calPathInfoGain_device<<<(pathPointsSize+63)/64,64>>>(pointCollection_dev,pathPoints_dev,pathInfoGain_tmp_dev,pathInfoGain_dev,pointCollectionSize,pathPointsSize);

    cudaStatus=cudaMemcpy(pathInfoGain,pathInfoGain_dev, sizeof(int)*pathPointsSize,cudaMemcpyDeviceToHost);
    if(cudaStatus !=cudaSuccess){
        fprintf(stderr, "pathPoints memory device to host failed");
        return;
    }

    cudaFree(pointCollection_dev);
    cudaFree(pathPoints_dev);
    cudaFree(pathInfoGain_dev);
    cudaFree(pathInfoGain_tmp_dev);
}
