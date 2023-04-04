#include "cuda_pointclouds.h"

__global__ void setPointLabelKernel(pcl::PointXYZRGBL *points, std::uint32_t label, int num_points) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_points) {
        points[idx].label = label;
    }
}

__global__ void transformPointKernel(pcl::PointXYZRGBL *points, Eigen::Matrix4d transform, int num_points) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_points) {
        Eigen::Vector4d p(points[idx].x, points[idx].y, points[idx].z, 1.0f);
        p = transform * p;
        points[idx].x = p(0);
        points[idx].y = p(1);
        points[idx].z = p(2);
    }
}