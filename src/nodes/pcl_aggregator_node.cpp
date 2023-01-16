/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* This node subscribes all the sensor data, and publishes the merged point cloud.
*/

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "PCLRegistrator.h"
#include "common.h"

#define POINTCLOUD_TOPIC "pointcloud"

#define MAX_POINTCLOUD_AGE 2

#define PCL_QUEUES_LEN 1000

int main(int argc, char **argv) {

    ROS_INFO("PointCloud aggregator node starting...");

    ros::init(argc, argv, "pcl_aggregator_node");

    ros::NodeHandle nh;

    int n_pointclouds, max_pointcloud_age;

    nh.param<int>("n_pointclouds", n_pointclouds, 1);
    nh.param<int>("max_pointcloud_age", max_pointcloud_age, MAX_POINTCLOUD_AGE);

    // allocate the subscribers
    ros::Subscriber *pcl_subscribers;
    if((pcl_subscribers = (ros::Subscriber *) malloc(n_pointclouds * sizeof(ros::Subscriber))) == NULL) {
        ROS_ERROR("Error allocating memory for pointcloud subscribers: %s", strerror(errno));
        return 1;
    }

    PCLRegistrator *registrator = new PCLRegistrator(n_pointclouds, max_pointcloud_age);

    // initialize the subscribers
    #pragma omp parallel for
    for(int i = 0; i < n_pointclouds; i++) {
        std::string topicName = POINTCLOUD_TOPIC + i;
        pcl_subscribers[i] = nh.subscribe<sensor_msgs::PointCloud2>(topicName, PCL_QUEUES_LEN, boost::bind(&PCLRegistrator::pointcloudCallback, registrator, _1, topicName));
        ROS_INFO("Subscribing to %s", topicName.c_str());
    }

    ROS_INFO("PointCloud aggregator node started.");

    ros::spin();

    return 0;
}