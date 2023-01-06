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
#include "common.h"

#define POINTCLOUD_TOPIC "/pointcloud"

#define PCL_PUBLISH_RATE 20 // pointcloud publish rate in Hz

#define RGBD_PCL_QUEUES_LEN 1000

void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {

}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {

}

void rgbdPclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

}

int main(int argc, char **argv) {

    ROS_INFO("PointCloud aggregator node starting...");

    ros::init(argc, argv, "pcl_aggregator_node");

    ros::NodeHandle nh;

    int n_pointclouds, pcl_publish_rate;

    nh.param<int>("n_pointclouds", n_pointclouds, 2); // TODO: allow dynamic number of pointclouds
    nh.param<int>("publish_rate", pcl_publish_rate, PCL_PUBLISH_RATE);

    // subscribe rgbd deprojector node
    ros::Subscriber rgbd_pcl_sub = nh.subscribe(POINTCLOUD_TOPIC, RGBD_PCL_QUEUES_LEN, rgbdPclCallback);

    ROS_INFO("PointCloud aggregator node started.");

    ros::spin();

    return 0;
}