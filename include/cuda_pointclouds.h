#ifndef CUDA_POINTCLOUDS_H_
#define CUDA_POINTCLOUDS_H_

#include <cuda_runtime.h>
#include <pcl/point_types.h>

__global__ void setPointLabelKernel(pcl::PointXYZRGBL *points, std::uint32_t label, int num_points);

#endif