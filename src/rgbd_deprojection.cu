/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* These are the CUDA kernel and functions used to deproject the RGBD image
* to a pointcloud with better performance and higher floating point precision.
*/

#include <iostream>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cstdint>
#include <eigen3/Eigen/Dense>

#define BLOCK_SIZE 512 // max block size on most modern GPUs is 1024

// this is the kernel. defined as __global__ so that it can be called from the host to run on the device
__global__ void deproject_pixel_to_point(uint8_t *frame, uint16_t width, uint16_t height, float fx, float fy, float cx, float cy, Eigen::Vector3f *points) {

    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if(x < width && y < height) {
        size_t index = y * width + x;
        float depth = frame[index];

        points[index][0] = (x - cx) * depth / fx;
        points[index][1] = (y - cy) * depth / fy;
        points[index][2] = depth;
    }
}

// this host function is called once from the rgbd_deprojector_node
// it treats everything device related and calls the kernel on the points
__host__ void deproject_frame(uint8_t *frame, uint16_t width, uint16_t height, Eigen::Matrix3f K, Eigen::Vector3f *points) {

    // determine the number of threads (and blocks) needed
    size_t n_pixels = width * height;
    size_t n_blocks = (size_t) ceil((float) n_pixels / BLOCK_SIZE);

    float fx, fy, cx, cy;
    fx = K(0, 0);
    fy = K(1, 1);
    cx = K(0, 2);
    cy = K(1, 2);


    // allocate space for the frame
    uint8_t *d_frame;
    cudaMalloc((void **) &d_frame, n_pixels * sizeof(uint8_t));

    // allocate space for the points
    Eigen::Vector3f *d_points;
    cudaMalloc((void **) &d_points, n_pixels * sizeof(Eigen::Vector3f));

    // copy the frame to the device
    cudaMemcpy(d_frame, frame, n_pixels * sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    // call the kernel
    deproject_pixel_to_point<<<n_blocks, BLOCK_SIZE>>>(d_frame, width, height, fx, fy, cx, cy, d_points);

    // wait for the kernel to finish
    cudaDeviceSynchronize();

    // copy the points to the host
    cudaMemcpy(points, d_points, n_pixels * sizeof(Eigen::Vector3f), cudaMemcpyDeviceToHost);

    // free the memory
    cudaFree(d_frame);
    cudaFree(d_points);
}