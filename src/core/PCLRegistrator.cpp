/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Do pointcloud registration from the multiple sources.
*/
#include "PCLRegistrator.h"

PCLRegistrator::PCLRegistrator(size_t n_sources, time_t max_pointcloud_age) {
    // get the number of sources and maximum pointcloud age
    this->n_sources = n_sources;
    this->max_pointcloud_age = max_pointcloud_age;

    // initialize the tf listener and buffer
    this->tfListener = std::make_shared<tf2_ros::TransformListener>(this->tfBuffer, true);

    // use this to initialize the sources manager
    this->initializeManager();
}

PCLRegistrator::~PCLRegistrator() {
}

// initialize the sources manager with the number of sources and configured max pointcloud age
void PCLRegistrator::initializeManager() {
    this->manager = std::make_shared<PointCloudsManager>(n_sources, max_pointcloud_age);
}

// called when any new pointcloud is received
void PCLRegistrator::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, std::string topicName) {

    ROS_INFO("New pointcloud from %s", topicName.c_str());

    // convert from the ROS to the PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    try {

        // get the transform from the robot base to the pointcloud frame
        geometry_msgs::TransformStamped transform =
                this->tfBuffer.lookupTransform(msg->header.frame_id, this->robotFrame, ros::Time(0));

        // convert tf to Eigen homogenous transformation matrix
        Eigen::Affine3d transformEigen;
        transformEigen = tf2::transformToEigen(transform);
        this->manager->setTransform(transformEigen, topicName);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("Error looking up the transform to '%s': %s", msg->header.frame_id.c_str(), ex.what());
        return;
    }

    // feed the manager with the new pointcloud
    this->manager->addCloud(cloud, topicName);
}

std::string PCLRegistrator::getRobotFrame() {
    return this->robotFrame;
}

void PCLRegistrator::setRobotFrame(std::string robotFrame) {
    this->robotFrame = robotFrame;
}

void PCLRegistrator::setPublisher(std::shared_ptr<ros::Publisher> point_cloud_pub) {
    this->point_cloud_pub = point_cloud_pub;
}

pcl::PointCloud<pcl::PointXYZ> PCLRegistrator::getPointCloud() {
    return this->manager->getMergedCloud();
}