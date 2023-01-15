#include "PCLStamped.h"

PCLStamped::PCLStamped(pcl::PointCloud<pcl::PointXYZ> *cloud) {
	this->addCloud(cloud);
}

PCLStamped::~PCLStamped() {
		delete this->cloud;
}

void PCLStamped::addCloud(pcl::PointCloud<pcl::PointXYZ> *cloud) {
	if(this->cloud != nullptr) {
		delete this->cloud;
	}
	this->cloud = cloud;
	if((this->timestamp = time(NULL)) < 0) {
		std::cerr << "Error getting current timestamp: " << strerror(errno) << std::endl;
		return;
	}
}

pcl::PointCloud<pcl::PointXYZ> *PCLStamped::getCloud() {
		return this->cloud;
}

time_t PCLStamped::getTimestamp() {
		return this->timestamp;
}
