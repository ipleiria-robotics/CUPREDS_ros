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
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "pcl/io/pcd_io.h"
#include "cupreds/SnapshotService.h"
#include "cupreds/StatisticsService.h"
#include "PCLRegistrator.h"
#include <memory>
#include <functional>

#define SUB_POINTCLOUD_TOPIC "pointcloud"
#define POINTCLOUD_TOPIC "merged_pointcloud"

#define MAX_POINTCLOUD_AGE 2
#define ROBOT_BASE "base_link"
#define AGGREGATOR_PUBLISH_RATE 10 // Hz

#define MAX_MEMORY 1000

#define NUM_SPINNER_THREADS 4
#define N_THREADS_IN_CALLBACK_POOL 32

#define PCL_QUEUES_LEN 50

void pointcloudPublishCallback(ros::Publisher* pub) {

    PCLRegistrator& registrator = PCLRegistrator::getInstance(0,0,0,0);
    pcl::PointCloud<pcl::PointXYZRGBL> pointcloud = registrator.getPointCloud();

    /*
    if(pointcloud.empty())
        return;

    if(pointcloud.points.size() != pointcloud.width * pointcloud.height)
        return;*/

    sensor_msgs::PointCloud2 ros_cloud;

    // convert the PCL pointcloud to the ROS PointCloud2 format
    try {
	    pcl::toROSMsg(pointcloud, ros_cloud);
    } catch (std::exception& e) {
        ROS_ERROR("Error converting pointcloud to ROS format: %s", e.what());
        return;
    }
	ros_cloud.header.frame_id = registrator.getRobotFrame();

	// publish the PointCloud
	pub->publish(ros_cloud);
}

void odomPublishCallback(tf2_ros::TransformBroadcaster& br, std::string& robot_base) {

    PCLRegistrator& registrator = PCLRegistrator::getInstance(0,0,0,0);

    // get the odometry value
    Eigen::Matrix4f odom = registrator.getOdom();
    
    // convert the Eigen::Matrix4f to a  Eigen::Affine3d
    Eigen::Affine3d odom_affine = Eigen::Affine3d(odom.cast<double>());

    // create the transform message
    geometry_msgs::TransformStamped tf_msg;
    tf_msg = tf2::eigenToTransform(odom_affine);
    tf_msg.header.stamp = ros::Time::now();
    // this defines the transform between the "odom" frame and the robot base link
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = robot_base;

    br.sendTransform(tf_msg);
}

bool handleSnapshotServiceRequest(cupreds::SnapshotService::Request& req, cupreds::SnapshotService::Response& res) {

    PCLRegistrator& registrator = PCLRegistrator::getInstance(0,0,0,0);

    pcl::PointCloud<pcl::PointXYZRGBL> pointcloud = registrator.getPointCloud();

    if(pointcloud.empty())
        return false;

    if(pointcloud.points.size() != pointcloud.width * pointcloud.height)
        return false;

    // save the pointcloud to a file
    std::string filename = req.input_filename;
    if(filename.empty())
        filename = "snapshot.pcd";

    try {
        pcl::io::savePCDFile(filename, pointcloud);
    } catch (std::exception& e) {
        ROS_ERROR("Error saving pointcloud to file: %s", e.what());
        return false;
    }

    res.output_filename = filename;

    return true;
}

bool handleStatisticsServiceRequest(cupreds::StatisticsService::Request& req, cupreds::StatisticsService::Response& res) {
    
    PCLRegistrator& registrator = PCLRegistrator::getInstance(0,0,0,0);

    res.intra_sensor_latency = registrator.getIntraSensorLatency();
    res.intra_sensor_stddev = registrator.getIntraSensorStdDev();

    res.inter_sensor_latency = registrator.getInterSensorLatency();
    res.inter_sensor_stddev = registrator.getInterSensorStdDev();

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

    // declare the point cloud publisher
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(POINTCLOUD_TOPIC, PCL_QUEUES_LEN);

    // declare the odometry tf publisher
    tf2_ros::TransformBroadcaster br;

	PCLRegistrator &registrator = PCLRegistrator::getInstance(n_pointclouds, max_pointcloud_age, max_memory, publish_rate);

    // initialize the publisher on the registrator
    registrator.setPublisher(pub);

    // set the robot base frame
    registrator.setRobotFrame(robot_base);

    // create a callback thread pool
    boost::asio::thread_pool pool(N_THREADS_IN_CALLBACK_POOL);

    // initialize the subscribers
    for(int i = 0; i < n_pointclouds; i++) {
        std::string topicName;
        topicName = "";
        topicName.append(SUB_POINTCLOUD_TOPIC);
        topicName.append(std::to_string(i));
        ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
            topicName, PCL_QUEUES_LEN,
            [&registrator, topicName, &pool](const sensor_msgs::PointCloud2::ConstPtr& msg) {
                registrator.pointcloudCallback(msg, topicName, &pool);
            }
        );
        pcl_subscribers.push_back(sub);
        ROS_INFO("Subscribing to %s", topicName.c_str());
    }

    // initialize the snapshot service
    ros::ServiceServer snapshot_service = nh.advertiseService("snapshot", handleSnapshotServiceRequest);

    // initialize the statistics service
    ros::ServiceServer statistics_service = nh.advertiseService("statistics", handleStatisticsServiceRequest);

    // start the spinner
    ros::AsyncSpinner spinner(NUM_SPINNER_THREADS, &callback_queue);
    spinner.start();

    ROS_INFO("PointCloud aggregator node started.");

    ros::Rate rate(publish_rate);
    while(ros::ok()) {
        pointcloudPublishCallback(&pub);
        odomPublishCallback(br, robot_base);
        rate.sleep();
    }

    // stop the spinner
	spinner.stop();

    return 0;
}
