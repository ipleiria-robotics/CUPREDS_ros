/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Do pointcloud registration from the multiple sources.
*/
#include "PCLRegistrator.h"

PCLRegistrator& PCLRegistrator::getInstance(size_t n_sources, double max_age, size_t max_memory, size_t publish_rate) {
    static PCLRegistrator instance(n_sources, max_age, max_memory, publish_rate);  // Static instance of the singleton
    return instance;
}

PCLRegistrator::PCLRegistrator(size_t n_sources, double max_pointcloud_age, size_t max_memory, size_t publish_rate) {
    // get the number of sources and maximum pointcloud age
    this->n_sources = n_sources;
    this->max_pointcloud_age = max_pointcloud_age;
    this->max_memory = max_memory;
    this->publish_rate = publish_rate;

    // initialize the tf listener and buffer
    this->tfListener = std::make_shared<tf2_ros::TransformListener>(this->tfBuffer, true);

    // use this to initialize the sources manager
    this->initializeManager();
}

PCLRegistrator::~PCLRegistrator() {
    this->manager.destruct();
}

// initialize the sources manager with the number of sources and configured max pointcloud age
void PCLRegistrator::initializeManager() {
    this->manager = pcl_aggregator::managers::InterSensorManager::get(n_sources, max_pointcloud_age, max_memory, publish_rate);
}

void pointcloudCallbackRoutine(const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& topicName) {
    // ROS_INFO("New pointcloud from %s", topicName.c_str());

    // convert from the ROS to the PCL format
    pcl::PointCloud<pcl::PointXYZ>::Ptr in (new pcl::PointCloud<pcl::PointXYZ>);
    try {
        pcl::fromROSMsg(*msg, *in);
    } catch (std::exception& e) {
        ROS_WARN("Error converting pointcloud from ROS to PCL: %s", e.what());
        return;
    }

    if(in->empty()) {
        in.reset();
        return;
    }

    // convert the input pointcloud from PointXYZ to PointXYZRGBL
    pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBL>);
    cloud->points.resize(in->points.size());

    // TODO: do this on CUDA
    #pragma omp for
    for (size_t i = 0; i < in->points.size(); ++i) {
        cloud->points[i].x = in->points[i].x;
        cloud->points[i].y = in->points[i].y;
        cloud->points[i].z = in->points[i].z;

        // Set RGB values to a constant color
        cloud->points[i].r = 255;
        cloud->points[i].g = 255;
        cloud->points[i].b = 255;

        // Set label to a constant value (e.g., 1)
        cloud->points[i].label = 1;
    }

    cloud->width = in->width;
    cloud->height = in->height;
    cloud->is_dense = in->is_dense;

    // destroy "in"
    in.reset();

    PCLRegistrator& pclRegistrator = PCLRegistrator::getInstance(0,0,0,0);

    try {

        // get the transform from the robot base to the pointcloud frame
        /*
        geometry_msgs::TransformStamped transform =
                this->tfBuffer.lookupTransform(msg->header.frame_id, this->robotFrame, ros::Time(0));
        */

        geometry_msgs::TransformStamped transform =
            pclRegistrator.tfBuffer.lookupTransform(msg->header.frame_id, pclRegistrator.robotFrame, ros::Time(0));

        // convert tf to Eigen homogenous transformation matrix
        Eigen::Affine3d transformEigen;
        transformEigen = tf2::transformToEigen(transform);
        // invert the affine transformation
        transformEigen.matrix().inverse();

        pclRegistrator.manager.setTransform(transformEigen, topicName);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("Error looking up the transform to '%s': %s", msg->header.frame_id.c_str(), ex.what());
        return;
    }

    // feed the manager with the new pointcloud
    pclRegistrator.manager.addCloud(std::move(cloud), topicName);
}

// called when any new pointcloud is received
void PCLRegistrator::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, const std::string& topicName, boost::asio::thread_pool *pool) {

    /*
    auto pointcloudRoutine = [] (PCLRegistrator *instance, const sensor_msgs::PointCloud2::ConstPtr& msg, std::string topicName) {
        pointcloudCallback(instance, msg, topicName);
    }*/

    // schedule the routine to the thread pool
    /*
    boost::asio::post(*pool, [msg, topicName]
    { return pointcloudCallbackRoutine(msg, topicName); });
     */

    /*
    // call the routine on a new thread
    std::thread pointcloudThread(pointcloudCallbackRoutine, this, msg, topicName);
    // dettach the thread to stop blocking execution
    pointcloudThread.detach();
    */

    pointcloudCallbackRoutine(msg, topicName);
}

std::string PCLRegistrator::getRobotFrame() {
    return this->robotFrame;
}

void PCLRegistrator::setRobotFrame(const std::string& robotFrame) {
    this->robotFrame = robotFrame;
}

void PCLRegistrator::setPublisher(const ros::Publisher& point_cloud_pub) {
    this->point_cloud_pub = point_cloud_pub;
}

pcl::PointCloud<pcl::PointXYZRGBL> PCLRegistrator::getPointCloud() {
    return this->manager.getMergedCloud();
}