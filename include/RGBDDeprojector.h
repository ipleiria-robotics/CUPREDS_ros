#ifndef RGBD_DEPROJECTOR_H
#define RGBD_DEPROJECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <eigen3/Eigen/Dense>
#include <cstring>
#include <cerrno>
#include <sys/time.h>

// CUDA function
extern "C" void deproject_frame(unsigned char *depth_image, int width, int height, Eigen::Matrix3f K, Eigen::Vector3f *points);

class RGBDDeprojector {

    private:
        Eigen::Matrix3f K;
        Eigen::Vector3f *points;

    public:
        RGBDDeprojector();
        ~RGBDDeprojector();

        bool allocPointsIfNotAllocated(size_t size);
        void freePoints(Eigen::Vector3f *points);

        Eigen::Matrix3f getK();
        void setK(Eigen::Matrix3f K);

        void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif