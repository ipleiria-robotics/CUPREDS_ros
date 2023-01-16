/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Deproject depth images to point clouds.
*/

#ifndef RGBD_DEPROJECTOR_H
#define RGBD_DEPROJECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/image_encodings.h"
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cstring>
#include <cerrno>
#include <sys/time.h>

class RGBDDeprojector {

    private:
        Eigen::Matrix<double, 3, 3> K;
        std::string camera_frame_id;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        ros::Publisher *point_cloud_pub = nullptr;
        cv::Mat *last_color_image = nullptr;

    public:
        RGBDDeprojector();
        ~RGBDDeprojector();

        Eigen::Matrix3d getK();
        void setK(Eigen::Matrix3d K);

        ros::Publisher getPointCloudPublisher();
        void setPointCloudPublisher(ros::Publisher *point_cloud_pub);

        void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);

        void colorImageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif