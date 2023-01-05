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

#define RGB_INFO_TOPIC "/camera/color/camera_info"
#define RGB_IMAGE_TOPIC "/camera/color/image_raw"

#define DEPTH_INFO_TOPIC "/camera/depth/camera_info"
#define DEPTH_IMAGE_TOPIC "/camera/depth/image_rect_raw"

#define POINTCLOUD_TOPIC "/pointcloud"

#define PCL_PUBLISH_RATE 20 // pointcloud publish rate in Hz

#define IMAGE_QUEUES_LEN 1000
#define INFO_QUEUES_LEN 1000
#define RGBD_PCL_QUEUES_LEN 1000

void rgbInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)  {

}

void rgbImageCallback(const sensor_msgs::Image::ConstPtr& msg) {

}

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

    std::string rgb_info_topic, rgb_image_topic;
    std::string depth_info_topic, depth_image_topic;
    std::string pointcloud_topic;
    int pcl_publish_rate;

    int n_pointclouds;

    // get the parameters, otherwise use the defaults
    nh.param<std::string>("rgb_info_topic", rgb_info_topic, RGB_INFO_TOPIC);
    nh.param<std::string>("rgb_image_topic", rgb_image_topic, RGB_IMAGE_TOPIC);
    nh.param<std::string>("depth_info_topic", depth_info_topic, DEPTH_INFO_TOPIC);
    nh.param<std::string>("depth_image_topic", depth_image_topic, DEPTH_IMAGE_TOPIC);
    nh.param<std::string>("pointcloud_topic", pointcloud_topic, POINTCLOUD_TOPIC);

    nh.param<int>("n_pointclouds", n_pointclouds, 2); // TODO: allow dynamic number of pointclouds

    nh.param<int>("publish_rate", pcl_publish_rate, PCL_PUBLISH_RATE);

    // subscribe RGB parameters and image
    ros::Subscriber rgb_info_sub = nh.subscribe(rgb_info_topic, INFO_QUEUES_LEN, rgbInfoCallback);
    ros::Subscriber rgb_image_sub = nh.subscribe(rgb_image_topic, IMAGE_QUEUES_LEN, rgbImageCallback);

    // subscribe depth parameters and image
    ros::Subscriber depth_info_sub = nh.subscribe(depth_info_topic, INFO_QUEUES_LEN, depthInfoCallback);
    ros::Subscriber depth_image_sub = nh.subscribe(depth_image_topic, IMAGE_QUEUES_LEN, depthImageCallback);

    // subscribe rgbd deprojector node
    ros::Subscriber rgbd_pcl_sub = nh.subscribe(pointcloud_topic, RGBD_PCL_QUEUES_LEN, rgbdPclCallback);

    ROS_INFO("PointCloud aggregator node started.");

    ros::spin();

    return 0;
}