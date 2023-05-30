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
#include "pcl/io/pcd_io.h"
#include "pcl_aggregator/SnapshotService.h"
#include "PCLRegistrator.h"
#include <memory>

#define SUB_POINTCLOUD_TOPIC "pointcloud"
#define POINTCLOUD_TOPIC "merged_pointcloud"

#define MAX_POINTCLOUD_AGE 2
#define ROBOT_BASE "base_link"
#define AGGREGATOR_PUBLISH_RATE 10 // Hz

#define MAX_MEMORY 5000

#define NUM_SPINNER_THREADS 16
#define N_THREADS_IN_CALLBACK_POOL 32

#define PCL_QUEUES_LEN 1000

void pointcloudPublishCallback(const ros::TimerEvent&, ros::Publisher* pub, PCLRegistrator *registrator) {

    pcl::PointCloud<pcl::PointXYZRGBL> pointcloud = registrator->getPointCloud();

    if(pointcloud.empty())
        return;

    if(pointcloud.points.size() != pointcloud.width * pointcloud.height)
        return;

    sensor_msgs::PointCloud2 ros_cloud;

    // convert the PCL pointcloud to the ROS PointCloud2 format
    try {
	    pcl::toROSMsg(pointcloud, ros_cloud);
    } catch (std::exception& e) {
        ROS_ERROR("Error converting pointcloud to ROS format: %s", e.what());
        return;
    }
	ros_cloud.header.frame_id = registrator->getRobotFrame();

	// publish the PointCloud
	pub->publish(ros_cloud);
}

bool handleSnapshotServiceRequest(pcl_aggregator::SnapshotService& req, pcl_aggregator::SnapshotService& res, 
    PCLRegistrator *registrator) {

    pcl::PointCloud<pcl::PointXYZRGBL> pointcloud = registrator->getPointCloud();

    if(pointcloud.empty())
        return false;

    if(pointcloud.points.size() != pointcloud.width * pointcloud.height)
        return false;

    // save the pointcloud to a file
    std::string filename = req.filename;
    if(filename.empty())
        filename = "snapshot.pcd";

    try {
        pcl::io::savePCDFile(filename, pointcloud);
    } catch (std::exception& e) {
        ROS_ERROR("Error saving pointcloud to file: %s", e.what());
        return false;
    }

    res.filename = filename;

    return true;
}

int main(int argc, char **argv) {

    ROS_INFO("PointCloud aggregator node starting...");

    ros::init(argc, argv, "pcl_aggregator_node");

    ros::NodeHandle nh;

    int n_pointclouds, publish_rate, max_memory;
    double max_pointcloud_age;
    std::string robot_base;

    nh.param<int>("n_pointclouds", n_pointclouds, 1);
    nh.param<double>("max_pointcloud_age", max_pointcloud_age, MAX_POINTCLOUD_AGE);
    nh.param<int>("publish_rate", publish_rate, AGGREGATOR_PUBLISH_RATE);
    nh.param<int>("max_memory", max_memory, MAX_MEMORY);
    nh.param<std::string>("robot_base", robot_base, ROBOT_BASE);

	ros::CallbackQueue callback_queue;
	nh.setCallbackQueue(&callback_queue);

    // allocate the subscribers
    std::vector<ros::Subscriber> pcl_subscribers;

	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_TOPIC, PCL_QUEUES_LEN);

	PCLRegistrator *registrator = new PCLRegistrator(n_pointclouds, max_pointcloud_age, max_memory);

    // initialize the publisher on the registrator
    registrator->setPublisher(pub);

    // set the robot base frame
    registrator->setRobotFrame(robot_base);

    // create a callback thread pool
    boost::asio::thread_pool pool(N_THREADS_IN_CALLBACK_POOL);

    // initialize the subscribers
    #pragma omp parallel for
    for(int i = 0; i < n_pointclouds; i++) {
        std::string topicName;
        topicName = "";
        topicName.append(SUB_POINTCLOUD_TOPIC);
        topicName.append(std::to_string(i));
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(topicName, PCL_QUEUES_LEN, boost::bind(&PCLRegistrator::pointcloudCallback, registrator, _1, topicName, &pool));
        pcl_subscribers.push_back(sub);
        ROS_INFO("Subscribing to %s", topicName.c_str());
    }

    // initialize the snapshot service
    ros::ServiceServer snapshot_service = nh.advertiseService<pcl_aggregator::SnapshotService, pcl_aggregator::SnapshotService::Request, pcl_aggregator::SnapshotService::Response>(
        "snapshot", boost::bind(&handleSnapshotServiceRequest, _1, _2, registrator));

    ROS_INFO("PointCloud aggregator node started.");

    // create a timer to call the publisher
    ros::Timer timer = nh.createTimer(ros::Duration(1.0 / publish_rate), boost::bind(&pointcloudPublishCallback, _1, &pub, registrator));

    // start the spinner
	ros::AsyncSpinner spinner(NUM_SPINNER_THREADS, &callback_queue);

	spinner.start();

    ros::waitForShutdown();

    // stop the spinner
	spinner.stop();

    delete registrator;

    return 0;
}
