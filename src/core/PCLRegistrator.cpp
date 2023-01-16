#include "PCLRegistrator.h"

PCLRegistrator::PCLRegistrator(size_t n_sources, time_t max_pointcloud_age) {
    // get the number of sources and maximum pointcloud age
    this->n_sources = n_sources;
    this->max_pointcloud_age = max_pointcloud_age;
    // use this to initialize the sources manager
    this->initializeManager();
}

PCLRegistrator::~PCLRegistrator() {
    // delete the sources manager instance to free all its internal buffers, etc
    delete manager;
}

// initialize the sources manager with the number of sources and configured max pointcloud age
void PCLRegistrator::initializeManager() {
    this->manager = new PointCloudsManager(n_sources, max_pointcloud_age);
}

// called when any new pointcloud is received
void PCLRegistrator::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, std::string topicName) {
    // convert from the ROS to the PCL format
    pcl::PointCloud<pcl::PointXYZ> *cloud = new pcl::PointCloud<pcl::PointXYZ>();
    pcl::fromROSMsg(*msg, *cloud);

    // feed the manager with the new pointcloud
    this->manager->addCloud(cloud, topicName);
}