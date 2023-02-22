/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the PointClouds received by a stream. Keeps the latest PointCloud received,
* registering the timestamp of the update and doing the transformation as set by the tf.
*/

#include "StreamManager.h"

#include <utility>

StreamManager::StreamManager() {
	this->cloud = nullptr;
	this->timestamp = -1;
    this->transformComputed = false;
}

StreamManager::~StreamManager() {
    this->cloud.reset();
}

void StreamManager::computeTransform() {
	// transform already computed, transform not set or no cloud
	if(this->transformComputed || !this->transformSet || this->cloud == nullptr) {
		return;
	}
	// do the transform
	pcl::transformPointCloud(*this->cloud, *this->cloud, this->transform);
	this->transformComputed = true; // set the flag to mark stream as ready
}

void StreamManager::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	this->transformComputed = false;
	this->cloud = std::move(cloud);
	this->timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	if(!this->transformSet) {
		ROS_WARN("Transform not set, cloud will not be transformed");
		return;
	}
	this->computeTransform();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StreamManager::getCloud() {
	return this->cloud;
}

time_t StreamManager::getTimestamp() {
	return this->timestamp;
}

void StreamManager::setTransform(Eigen::Affine3d transform) {
	// mark the transform as not computed
	this->transformComputed = false;

	// set the new transform
    this->transform = transform;
    this->transformSet = true;
	this->computeTransform();
}

bool StreamManager::hasCloudReady() {
	return this->cloud != nullptr && this->transformComputed;
}