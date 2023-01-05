/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* This node constantly receives the depth and camera info to deal with deprojection.
* It publishes a deprojected point cloud.
*/

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <eigen3/Eigen/Dense>
#include <cuda.h>
#include <cuda_runtime.h>

#define THREADS_PER_BLOCK 512

eigen::Matrix3f K = Eigen::Matrix3f::Zero();

void depthInfoCallback(sensor_msgs::CameraInfo::ConstPtr& msg) {

    /* We only use the intrinsics matrix.
    * It doesn't change out of nowhere, only need to store the first message.
    */

   if(K != Eigen::Matrix3f::Zero())
       return;
    
   K = msg->K;
}

void depthImageCallback(sensor_msgs::Image::ConstPtr& msg) {

    size_t n_pixels = msg->width * msg->height;

    // allocate memory for the depth image
    uint8_t *d_frame;
    if(cudaMalloc((void**)&d_frame, n_pixels * sizeof(uint8_t)) != cudaSuccess) {
        ROS_ERROR("Failed to allocate memory for the depth image.");
        return;
    }

    // allocate memory for the deprojected point cloud
    eigen::Vector3f *d_pcl;
    if(cudaMalloc((void**)&d_pcl, n_pixels * sizeof(eigen::Vector3f)) != cudaSuccess) {
        ROS_ERROR("Failed to allocate memory for the deprojected point cloud.");
        return;
    
    }

    // copy the depth image to the device
    if(cudaMemcpy(d_frame, msg->data, n_pixels * sizeof(uint8_t), cudaMemcpyHostToDevice) != cudaSuccess) {
        ROS_ERROR("Failed to copy the depth image to the device.");
        return;
    }

    // call the CUDA kernel
    size_t n_blocks = (n_pixels + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
    deproject_pixel<<<n_blocks, THREADS_PER_BLOCK>>>(msg->data, msg->width, msg->height, K(0, 0), K(1, 1), K(0, 2), K(1, 2), d_pcl);

    // wait for the device
    if(cudaDeviceSynchronize() != cudaSuccess) {
        ROS_ERROR("Failed to synchronize the device.");
        return;
    }

    // copy the deprojected point cloud to the host
    eigen::Vector3f *h_pcl = (eigen::Vector3f*)malloc(n_pixels * sizeof(eigen::Vector3f));
    if(cudaMemcpy(h_pcl, d_pcl, n_pixels * sizeof(eigen::Vector3f), cudaMemcpyDeviceToHost) != cudaSuccess) {
        ROS_ERROR("Failed to copy the deprojected point cloud to the host.");
        return;
    }

    // free the device memory
    cudaFree(d_frame);
    cudaFree(d_pcl);

}

int main(int argc, char **argv) {

    // TODO: subscribe depth camera info
    // TODO: subscribe depth camera image
    // TODO: publish deprojected point cloud
    
}
