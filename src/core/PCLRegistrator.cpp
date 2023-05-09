/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Do pointcloud registration from the multiple sources.
*/
#include "PCLRegistrator.h"

PCLRegistrator::PCLRegistrator(size_t n_sources, double max_pointcloud_age) {
    // get the number of sources and maximum pointcloud age
    this->n_sources = n_sources;
    this->max_pointcloud_age = max_pointcloud_age;

    // initialize the tf listener and buffer
    this->tfListener = std::make_shared<tf2_ros::TransformListener>(this->tfBuffer, true);

    // use this to initialize the sources manager
    this->initializeManager();
}

PCLRegistrator::~PCLRegistrator() {
    this->manager.reset();
    this->tfListener.reset();
}

// initialize the sources manager with the number of sources and configured max pointcloud age
void PCLRegistrator::initializeManager() {
    this->manager = std::make_shared<PointCloudsManager>(n_sources, max_pointcloud_age);
}

void pointcloudCallbackRoutine(PCLRegistrator* pclRegistrator, const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& topicName) {
    ROS_INFO("New pointcloud from %s", topicName.c_str());

    // convert from the ROS to the PCL format
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBL>);
    try {
        pcl::fromROSMsg(*msg, *cloud);
    } catch (std::exception& e) {
        ROS_WARN("Error converting pointcloud from ROS to PCL: %s", e.what());
        return;
    }

    if(cloud->empty()) {
        cloud.reset();
        return;
    }

    try {

        // get the transform from the robot base to the pointcloud frame
        /*
        geometry_msgs::TransformStamped transform =
                this->tfBuffer.lookupTransform(msg->header.frame_id, this->robotFrame, ros::Time(0));
        */

        geometry_msgs::TransformStamped transform =
            pclRegistrator->tfBuffer.lookupTransform(msg->header.frame_id, pclRegistrator->robotFrame, ros::Time(0));

        // convert tf to Eigen homogenous transformation matrix
        Eigen::Affine3d transformEigen;
        transformEigen = tf2::transformToEigen(transform);
        // invert the affine transformation
        transformEigen.matrix().inverse();

        pclRegistrator->manager->setTransform(transformEigen, topicName);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("Error looking up the transform to '%s': %s", msg->header.frame_id.c_str(), ex.what());
        return;
    }

    // feed the manager with the new pointcloud
    pclRegistrator->manager->addCloud(std::move(cloud), topicName);
}

// called when any new pointcloud is received
void PCLRegistrator::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& topicName, boost::asio::thread_pool* pool) {

    boost::asio::post(*pool, [this, msg, topicName]
    { return pointcloudCallbackRoutine(this, msg, topicName); });

    // pointcloudCallbackRoutine(this, msg, topicName);

    /*
    // call the routine on a new thread
    std::thread pointcloudThread(pointcloudCallbackRoutine, this, msg, topicName);
    // dettach the thread to stop blocking execution
    pointcloudThread.detach();
    */
}

std::string PCLRegistrator::getRobotFrame() {
    return this->robotFrame;
}

void PCLRegistrator::setRobotFrame(const std::string& robotFrame) {
    this->robotFrame = robotFrame;
}

void PCLRegistrator::setPublisher(ros::Publisher point_cloud_pub) {
    this->point_cloud_pub = point_cloud_pub;
}

pcl::PointCloud<pcl::PointXYZRGBL> PCLRegistrator::getPointCloud() {
    return this->manager->getMergedCloud();
}