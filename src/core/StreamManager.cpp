/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the PointClouds received by a stream. Keeps the latest PointCloud received,
* registering the timestamp of the update and doing the transformation as set by the tf.
*/

#include "StreamManager.h"

StreamManager::StreamManager() {
	this->cloud = nullptr;
	this->timestamp = -1;
}

StreamManager::~StreamManager() {
	delete &(this->cloud);
	delete this->transform;
}

void StreamManager::computeTransform() {
	// transform already computed, transform not set or no cloud
	if(this->transformComputed || this->transform == nullptr || this->cloud == nullptr) {
		return;
	}
	// do the transform
	pcl::transformPointCloud(*this->cloud, *this->cloud, *this->transform);
	this->transformComputed = true; // set the flag to mark stream as ready
}

void StreamManager::addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	this->transformComputed = false;
	// if a cloud was already set, delete the instance to avoid memory leaks
	if(this->cloud != nullptr) {
		delete &(this->cloud);
	}
	this->cloud = cloud;
	if((this->timestamp = time(NULL)) < 0) {
		std::cerr << "Error getting current timestamp: " << strerror(errno) << std::endl;
		return;
	}
	if(this->transform == nullptr) {
		std::cerr << "Transform not set, cloud will not be transformed";
		return;
	}
	this->computeTransform();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr StreamManager::getCloud() {
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

bool StreamManager::hasCloudReady() {
	return this->cloud != nullptr && this->transformComputed;
}