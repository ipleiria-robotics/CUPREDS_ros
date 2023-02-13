/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* This node constantly receives the depth and camera info to deal with deprojection.
* It publishes a deprojected point cloud.
*/

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <eigen3/Eigen/Dense>
#include "RGBDDeprojector.h"

#define DEPTH_INFO_TOPIC "/camera/aligned_depth_to_color/camera_info"
#define DEPTH_IMAGE_TOPIC "/camera/aligned_depth_to_color/image_raw"
#define COLOR_IMAGE_TOPIC "/camera/color/image_raw"

#define INFO_QUEUES_LEN 1000
#define IMAGE_QUEUES_LEN 1000
#define PCL_QUEUES_LEN 1000

#define DEPROJECTOR_PUBLISH_RATE 30

int main(int argc, char **argv) {

    ROS_INFO("RGBD deprojector node starting...");

    ros::init(argc, argv, "rgbd_deprojector_node");

    ros::NodeHandle nh;
    std::string depth_info_topic, depth_image_topic, color_image_topic;
    int publish_rate;

    // get the configuration params
    nh.param<std::string>("depth_info_topic", depth_info_topic, DEPTH_INFO_TOPIC);
    nh.param<std::string>("depth_image_topic", depth_image_topic, DEPTH_IMAGE_TOPIC);
    nh.param<std::string>("color_image_topic", color_image_topic, COLOR_IMAGE_TOPIC);
    nh.param<int>("publish_rate", publish_rate, DEPROJECTOR_PUBLISH_RATE);

    // instantiate a RGBD deprojector object
    RGBDDeprojector *rgbd_deprojector = new RGBDDeprojector();

    // subscribe depth parameters and image
    ros::Subscriber depth_info_sub = nh.subscribe(DEPTH_INFO_TOPIC, INFO_QUEUES_LEN, &RGBDDeprojector::depthInfoCallback, rgbd_deprojector);
    ROS_INFO("Subscribing depth camera info");

    ros::Subscriber depth_image_sub = nh.subscribe(DEPTH_IMAGE_TOPIC, IMAGE_QUEUES_LEN, &RGBDDeprojector::depthImageCallback, rgbd_deprojector);
    ROS_INFO("Subscribing depth camera image");

    ros::Subscriber color_image_sub = nh.subscribe(COLOR_IMAGE_TOPIC, IMAGE_QUEUES_LEN, &RGBDDeprojector::colorImageCallback, rgbd_deprojector);
    ROS_INFO("Subscribing color camera image");

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", PCL_QUEUES_LEN);
    std::shared_ptr<ros::Publisher> pointcloud_pub(&pub);
    rgbd_deprojector->setPointCloudPublisher(pointcloud_pub);
    
    ros::spin();

    return 0;
}
