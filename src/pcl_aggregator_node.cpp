/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*/

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

#define RGB_INFO_TOPIC "/camera/color/camera_info"
#define RGB_IMAGE_TOPIC "/camera/color/image_raw"

#define DEPTH_INFO_TOPIC "/camera/depth/camera_info"
#define DEPTH_IMAGE_TOPIC "/camera/depth/image_rect_raw"

#define IMAGE_QUEUES_LEN 1000
#define INFO_QUEUES_LEN 1000

void rgbInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)  {

}

void rgbImageCallback(const sensor_msgs::Image::ConstPtr& msg) {

}

void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {

}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {

}

int main(int argc, char **argv) {

    ROS_INFO("PointCloud aggregator node starting...");

    ros::init(argc, argv, "pcl_aggregator_node");

    ros::NodeHandle nh;

    // subscribe RGB parameters and image
    ros::Subscriber rgb_info_sub = nh.subscribe(RGB_INFO_TOPIC, INFO_QUEUES_LEN, rgbInfoCallback);
    ros::Subscriber rgb_image_sub = nh.subscribe(RGB_IMAGE_TOPIC, IMAGE_QUEUES_LEN, rgbImageCallback);

    // subscribe depth parameters and image
    ros::Subscriber depth_info_sub = nh.subscribe(DEPTH_INFO_TOPIC, INFO_QUEUES_LEN, depthInfoCallback);
    ros::Subscriber depth_image_sub = nh.subscribe(DEPTH_IMAGE_TOPIC, IMAGE_QUEUES_LEN, depthImageCallback);

    ROS_INFO("PointCloud aggregator node started.");

    ros::spin();

    return 0;
}