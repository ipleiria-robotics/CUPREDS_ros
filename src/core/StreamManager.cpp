/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the PointClouds received by a stream. Keeps the latest PointCloud received,
* registering the timestamp of the update and doing the transformation as set by the tf.
*/

#include "StreamManager.h"

#include <utility>

StreamManager::StreamManager(std::string topicName) {
    this->topicName = topicName;
	this->cloud = nullptr;
	this->timestamp = -1;
    this->sensorTransformComputed = false;
}

StreamManager::~StreamManager() {
    this->cloud.reset();
}

// comparators implementation
// streammanagers are compared by the latest timestamp.
// they are equal if they refer to the same sensor
bool StreamManager::operator<(const StreamManager &other) const {
    return this->timestamp < other.timestamp;
}

bool StreamManager::operator>(const StreamManager &other) const {
    return this->timestamp > other.timestamp;
}

bool StreamManager::operator==(const StreamManager &other) const {
    return this->topicName == other.topicName;
}

void StreamManager::computeTransform() {
	// transform already computed, transform not set or no cloud
	if(this->sensorTransformComputed || !this->sensorTransformSet || this->cloud == nullptr) {
		return;
	}
	// do the transform
	pcl::transformPointCloud(*this->cloud, *this->cloud, this->sensorTransform);
	this->sensorTransformComputed = true; // set the flag to mark stream as ready
}

// this is a routine to call from a thread to transform a pointcloud
void applyTransformRoutine(StampedPointCloud spcl, Eigen::Affine3d tf) {
	spcl.applyTransform(tf);
}

// this is a routine to call from a thread to clear the pointclouds which don't meet the criteria
void clearPointCloudsRoutine(StreamManager *instance) {
	instance->clear();
}

void StreamManager::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	// create a stamped point cloud object to keep this pointcloud
	StampedPointCloud spcl = StampedPointCloud();
	spcl.setOriginTopic(this->topicName);
	spcl.setPointCloud(cloud);

	// start a thread to transform the pointcloud
	std::thread transformationThread(applyTransformRoutine, spcl, sensorTransform);

	// start a thread to clear the pointclouds older than max age
	std::thread cleaningThread(clearPointCloudsRoutine, this);

	// wait for both threads to synchronize
	transformationThread.join();
	cleaningThread.join();

	// add the new pointcloud to the set
	this->clouds.insert(spcl);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StreamManager::getCloud() {

	// clear the old cloud
	this->cloud->empty();

	// clear the set
	this->clear();

	// create a icp object
	pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;

	// iterator
	std::set<StampedPointCloud,CompareStampedPointCloud>::iterator it;

	// iterate over all pointclouds in the set and do ICP
	for(it = this->clouds.begin(); it != this->clouds.end(); ++it) {

		// check if the pointcloud is transformed
		if(!(it->isTransformComputed())) {
			// compute the transform if available
			if(this->sensorTransformSet) {
				pcl::transformPointCloud<pcl::PointXYZRGB>(*it->getPointCloud(), *it->getPointCloud(),
					this->sensorTransform);
			}
		}

		if(it->isTransformComputed()) {
			icp.setInputSource(it->getPointCloud());
			icp.setInputTarget(this->cloud);
			icp.align(*this->cloud);
		}

	}
	return this->cloud;
}

time_t StreamManager::getTimestamp() {
	return this->timestamp;
}

void StreamManager::setTimestamp(long long timestamp) {
	this->timestamp = timestamp;
}

void StreamManager::setSensorTransform(Eigen::Affine3d transform) {
	// mark the transform as not computed
	this->sensorTransformComputed = false;

	// set the new transform
    this->sensorTransform = transform;
    this->sensorTransformSet = true;
	this->computeTransform();
}

void StreamManager::setMaxAge(time_t max_age) {
	this->max_age = max_age;
}

time_t StreamManager::getMaxAge() {
	return this->max_age;
}

void StreamManager::clear() {

	// create a comparison object
	StampedPointCloud spc_comp = StampedPointCloud();
	long long max_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	max_timestamp -= (this->max_age * 1000);
	spc_comp.setTimestamp(max_timestamp);
	
	// find the first pointcloud not meeting criteria
	auto iter = this->clouds.lower_bound(spc_comp);
	// remove all pointclouds not meeting the criteria
	while(iter != this->clouds.begin()) {
		--iter;
		this->clouds.erase(iter);
	}
}

bool StreamManager::hasCloudReady() {
	return this->cloud != nullptr && this->sensorTransformComputed;
}
