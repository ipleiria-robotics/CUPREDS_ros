#include <cuda_rgbd.h>

__global__ void deprojectImageKernel(std::uint8_t* depth, std::uint8_t* color, pcl::PointXYZRGB* cloud, int width, int height, float fx, float fy, float cx, float cy) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    int idx = y * width + x;
    float z = depth[idx];
    
    cloud[idx].x = (x - cx) * z / fx;
    cloud[idx].y = (y - cy) * z / fy;
    cloud[idx].z = z;
    cloud[idx].r = color[idx * 3];
    cloud[idx].g = color[idx * 3 + 1];
    cloud[idx].b = color[idx * 3 + 2];
}
