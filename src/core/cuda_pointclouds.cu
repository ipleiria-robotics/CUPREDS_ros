#include "cuda_pointclouds.h"

__global__ void setPointLabelKernel(pcl::PointXYZRGBL *points, std::uint32_t label, int num_points)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < num_points)
    {
        points[idx].label = label;
    }
}