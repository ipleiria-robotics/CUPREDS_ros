#include <cuda_rgbd.h>

__global__ void deprojectImageKernel(std::uint8_t* depth, std::uint8_t* color, pcl::PointXYZRGB* cloud, int width, int height, Eigen::Matrix3d K, Eigen::Matrix4d Rt) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    int idx = y * width + x;

    float z = depth[idx];

    // pixel as vector
    Eigen::Vector3d p(x, y, z);

    // deproject pixel
    Eigen::Vector3d p3d = K.inverse() * Rt.inverse() * p;
    
    // fill the point
    cloud[idx].x = p3d[0];
    cloud[idx].y = p3d[1];
    cloud[idx].z = p3d[2];
    cloud[idx].r = color[idx * 3];
    cloud[idx].g = color[idx * 3 + 1];
    cloud[idx].b = color[idx * 3 + 2];
}
