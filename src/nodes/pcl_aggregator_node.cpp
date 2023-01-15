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

#define PCL_QUEUES_LEN 1000

void pclCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
}

int main(int argc, char **argv) {

    ROS_INFO("PointCloud aggregator node starting...");

    ros::init(argc, argv, "pcl_aggregator_node");

    ros::NodeHandle nh;

    int n_pointclouds, pcl_publish_rate;

    nh.param<int>("n_pointclouds", n_pointclouds, 2); // TODO: allow dynamic number of pointclouds
    nh.param<int>("publish_rate", pcl_publish_rate, PCL_PUBLISH_RATE);

    // allocate the subscribers
    ros::Subscriber *pcl_subscribers;
    if((pcl_subscribers = (ros::Subscriber *) malloc(n_pointclouds * sizeof(ros::Subscriber))) == NULL) {
        ROS_ERROR("Error allocating memory for pointcloud subscribers: %s", strerror(errno));
        return 1;
    }
    // initialize the subscribers
    for(int i = 0; i < n_pointclouds; i++) {
        pcl_subscribers[i] = nh.subscribe(POINTCLOUD_TOPIC, PCL_QUEUES_LEN, pclCallback);
    }

    ROS_INFO("PointCloud aggregator node started.");

    ros::spin();

    return 0;
}