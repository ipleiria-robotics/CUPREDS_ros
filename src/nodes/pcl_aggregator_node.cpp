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
#include "pcl_conversions/pcl_conversions.h"
#include "tf/transform_listener.h"
#include "PCLRegistrator.h"
#include <memory>

#define SUB_POINTCLOUD_TOPIC "pointcloud"
#define POINTCLOUD_TOPIC "merged_pointcloud"

#define MAX_POINTCLOUD_AGE 2
#define ROBOT_BASE "base_link"
#define AGGREGATOR_PUBLISH_RATE 10 // Hz

#define NUM_SPINNER_THREADS 16

#define PCL_QUEUES_LEN 1000

void pointcloudPublishCallback(const ros::TimerEvent&, ros::Publisher* pub, PCLRegistrator *registrator) {
    sensor_msgs::PointCloud2 ros_cloud;

    // convert the PCL pointcloud to the ROS PointCloud2 format
	pcl::toROSMsg(registrator->getPointCloud(), ros_cloud);
	ros_cloud.header.frame_id = registrator->getRobotFrame();

	// publish the PointCloud
	pub->publish(ros_cloud);
}

int main(int argc, char **argv) {

    ROS_INFO("PointCloud aggregator node starting...");

    ros::init(argc, argv, "pcl_aggregator_node");

    ros::NodeHandle nh;

    int n_pointclouds, publish_rate;
    double max_pointcloud_age;
    std::string robot_base;

    nh.param<int>("n_pointclouds", n_pointclouds, 1);
    nh.param<double>("max_pointcloud_age", max_pointcloud_age, MAX_POINTCLOUD_AGE);
    nh.param<int>("publish_rate", publish_rate, AGGREGATOR_PUBLISH_RATE);
    nh.param<std::string>("robot_base", robot_base, ROBOT_BASE);

	ros::CallbackQueue callback_queue;
	nh.setCallbackQueue(&callback_queue);

    // allocate the subscribers
    std::vector<ros::Subscriber> pcl_subscribers;

	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_TOPIC, PCL_QUEUES_LEN);

	PCLRegistrator *registrator = new PCLRegistrator(n_pointclouds, max_pointcloud_age);

    // initialize the publisher on the registrator
    registrator->setPublisher(pub);

    // set the robot base frame
    registrator->setRobotFrame(robot_base);

    // initialize the subscribers
    #pragma omp parallel for
    for(int i = 0; i < n_pointclouds; i++) {
        std::string topicName;
        topicName = "";
        topicName.append(SUB_POINTCLOUD_TOPIC);
        topicName.append(std::to_string(i));
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topicName, PCL_QUEUES_LEN, boost::bind(&PCLRegistrator::pointcloudCallback, registrator, _1, topicName));
        pcl_subscribers.push_back(sub);
        ROS_INFO("Subscribing to %s", topicName.c_str());
    }

    ROS_INFO("PointCloud aggregator node started.");

    // create a timer to call the publisher
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / publish_rate), boost::bind(&pointcloudPublishCallback, _1, &pub, registrator));

    // start the spinner
	ros::AsyncSpinner spinner(NUM_SPINNER_THREADS, &callback_queue);
	spinner.start();

    // spin
    ros::waitForShutdown();

    // stop the spinner
	spinner.stop();

    delete registrator;

    return 0;
}
