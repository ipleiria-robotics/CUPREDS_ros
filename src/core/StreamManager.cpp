#include "StreamManager.h"

StreamManager::StreamManager(pcl::PointCloud<pcl::PointXYZ> *cloud) {
	this->addCloud(cloud);
}

StreamManager::~StreamManager() {
	delete this->cloud;
}

void StreamManager::addCloud(pcl::PointCloud<pcl::PointXYZ> *cloud) {
	if(this->cloud != nullptr) {
		delete this->cloud;
	}
	this->cloud = cloud;
	if((this->timestamp = time(NULL)) < 0) {
		std::cerr << "Error getting current timestamp: " << strerror(errno) << std::endl;
		return;
	}
}

pcl::PointCloud<pcl::PointXYZ> *StreamManager::getCloud() {
	return this->cloud;
}

time_t StreamManager::getTimestamp() {
	return this->timestamp;
}
