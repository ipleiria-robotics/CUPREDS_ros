/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Do pointcloud registration from the multiple sources.
*/

#ifndef PCL_REGISTRATION_
#define PCL_REGISTRATION_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"
#include "PointCloudsManager.h"
#include <memory>

class PCLRegistrator {

    private:
        std::shared_ptr<PointCloudsManager> manager = nullptr;
        size_t n_sources;
        double max_pointcloud_age;
        std::string robotFrame = "base_link";
		ros::Publisher point_cloud_pub;
        tf2_ros::Buffer tfBuffer;
		std::shared_ptr<tf2_ros::TransformListener> tfListener = nullptr;
        void initializeManager();

    public:
        PCLRegistrator(size_t n_sources, double max_age);
        ~PCLRegistrator();
        void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, std::string topicName);
        std::string getRobotFrame();
        void setRobotFrame(std::string robotFrame);
        void setPublisher(ros::Publisher point_cloud_pub);
        pcl::PointCloud<pcl::PointXYZRGB> getPointCloud();
};

#endif
