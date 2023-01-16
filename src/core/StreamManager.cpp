#include "StreamManager.h"

StreamManager::StreamManager() {
	this->cloud = nullptr;
	this->timestamp = 0;
}

StreamManager::~StreamManager() {
	delete this->cloud;
	delete this->transform;
}

void StreamManager::computeTransform() {
	if(this->transformComputed || this->transform == nullptr || this->cloud == nullptr) {
		return;
	}
	pcl::transformPointCloud(*this->cloud, *this->cloud, *this->transform);
	this->transformComputed = true;
}

void StreamManager::addCloud(pcl::PointCloud<pcl::PointXYZ> *cloud) {
	this->transformComputed = false;
	if(this->cloud != nullptr) {
		delete this->cloud;
	}
	this->cloud = cloud;
	if((this->timestamp = time(NULL)) < 0) {
		std::cerr << "Error getting current timestamp: " << strerror(errno) << std::endl;
		return;
	}
	this->computeTransform();
}

pcl::PointCloud<pcl::PointXYZ> *StreamManager::getCloud() {
	return this->cloud;
}

time_t StreamManager::getTimestamp() {
	return this->timestamp;
}

void StreamManager::setTransform(Eigen::Affine3d *transform) {
	// mark the transform as not computed
	this->transformComputed = false;

	// delete existing transform
	if(this->transform != nullptr) {
		delete this->transform;
	}

	// set the new transform
	this->transform = transform;
	this->computeTransform();
}