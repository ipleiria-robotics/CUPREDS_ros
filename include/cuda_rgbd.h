#ifndef CUDA_RGBD_H_
#define CUDA_RGBD_H_

#include <cuda_runtime.h>

__global__ void deprojectImageKernel(std::uint8_t* depth, std::uint8_t* color, pcl::PointXYZRGB* cloud, int width, int height, float fx, float fy, float cx, float cy);

#endif