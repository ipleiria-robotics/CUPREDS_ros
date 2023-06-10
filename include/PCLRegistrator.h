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
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.h"
#include <pcl_aggregator_core/managers/PointCloudsManager.h>
#include <memory>
#include <thread>
#include <boost/asio/thread_pool.hpp>
#include <boost/asio/post.hpp>

class PCLRegistrator
{

    private:
        std::shared_ptr<pcl_aggregator::managers::PointCloudsManager> manager = nullptr;
        size_t n_sources;
        double max_pointcloud_age;
        size_t max_memory;
        std::string robotFrame = "base_link";
        ros::Publisher point_cloud_pub;
        tf2_ros::Buffer tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener = nullptr;

        PCLRegistrator(size_t n_sources, double max_age, size_t max_memory);
        ~PCLRegistrator();

        void initializeManager();

    public:
        static PCLRegistrator &getInstance(size_t n_sources, double max_age, size_t max_memory);

        void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, std::string topicName, boost::asio::thread_pool *pool);
        std::string getRobotFrame();
        void setRobotFrame(const std::string &robotFrame);
        void setPublisher(ros::Publisher point_cloud_pub);
        pcl::PointCloud<pcl::PointXYZRGBL> getPointCloud();

        // Disable copy construction and assignment operator
        PCLRegistrator(const PCLRegistrator&) = delete;
        PCLRegistrator& operator=(const PCLRegistrator&) = delete;

    friend void pointcloudCallbackRoutine(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& topicName);
};

#endif
