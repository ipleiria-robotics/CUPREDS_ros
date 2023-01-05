/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* These are the CUDA kernel and functions used to deproject the RGBD image
* to a pointcloud with better performance and higher floating point precision.
*/

#include <cuda.h>
#include <cuda_runtime.h>
#include <stdint.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

__global__ void deproject_pixel(uint8_t *frame, uint16_t width, uint16_t height, float fx, float fy, float cx, float cy, eigen::Vector3f *point) {

    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if(x < width && y < height) {
        size_t index = y * width + x;
        float depth = frame[index];

        point[0] = (x - cx) * depth / fx;
        point[1] = (y - cy) * depth / fy;
        point[2] = depth;
    }
}