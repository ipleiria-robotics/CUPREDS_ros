#ifndef RGBD_DEPROJECTOR_H
#define RGBD_DEPROJECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cstring>
#include <cerrno>
#include <sys/time.h>

#define BLOCK_SIZE 512

// CUDA function
extern "C" __global__ void cuda_deproject_frame(uint8_t *frame, uint16_t width, uint16_t height, float fx, float fy, float cx, float cy, Eigen::Vector3f *cloud);

class RGBDDeprojector {

    private:
        Eigen::Matrix3f K;
        Eigen::Vector3f *points;
        Eigen::Vector3f *d_points;
        uint8_t *d_frame;
        ros::Publisher *point_cloud_pub = nullptr;

    public:
        RGBDDeprojector();
        ~RGBDDeprojector();

        // host pointcloud allocation and deallocation
        bool allocPointsIfNotAllocated(size_t size);
        void freePoints();

        // device buffers allocation and deallocation
        bool allocDevicePointsIfNotAllocated(size_t size);
        bool allocDeviceFrameIfNotAllocated(size_t size);
        void freeDevicePoints();
        void freeDeviceFrame();

        Eigen::Matrix3f getK();
        void setK(Eigen::Matrix3f K);

        ros::Publisher getPointCloudPublisher();
        void setPointCloudPublisher(ros::Publisher *point_cloud_pub);

        void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif