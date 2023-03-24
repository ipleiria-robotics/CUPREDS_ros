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
	this->cloud = pcl::PointCloud<pcl::PointXYZRGB>().makeShared();
	this->timestamp = -1;
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

	while(this->clouds_not_transformed.size() > 0) {
		
		// get the first element
		StampedPointCloud spcl = this->clouds_not_transformed.front();
		spcl.applyTransform(this->sensorTransform);

		// add to the set
		this->clouds.insert(spcl);

		// remove from the queue
		this->clouds_not_transformed.pop();
	}

	this->pointCloudSet = true;
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

	if(this->sensorTransformSet) {
		// transform the incoming pointcloud and add directly to the set

		/*
		// start a thread to transform the pointcloud
		std::thread transformationThread(applyTransformRoutine, spcl, sensorTransform);

		// start a thread to clear the pointclouds older than max age
		std::thread cleaningThread(clearPointCloudsRoutine, this);
		*/

		// this->clear();
		spcl.applyTransform(this->sensorTransform);

		// wait for both threads to synchronize
		/*
		transformationThread.join();
		cleaningThread.join();
		*/

		// add the new pointcloud to the set
		this->clouds.insert(spcl);

	} else {

		// add the pointcloud to the queue
		this->clouds_not_transformed.push(spcl);
	}
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

	bool firstCloud = true;

	// iterate over all pointclouds in the set and do ICP
	for(it = this->clouds.begin(); it != this->clouds.end(); ++it) {
		if(firstCloud) {
			// copy the first pointcloud to the cloud
			pcl::copyPointCloud(*it->getPointCloud(), *this->cloud);
			firstCloud = false;
			continue;
		}
		try {
			if(it->getPointCloud()->size() == 0)
				continue;
			icp.setInputSource(it->getPointCloud());
			icp.setInputTarget(this->cloud);
			icp.align(*this->cloud);

		} catch (std::exception &e) {
			std::cout << "Error performing sensor-wise ICP: " << e.what() << std::endl;
		}
	}

	this->pointCloudSet = true;
	return this->cloud;
}

long long StreamManager::getTimestamp() {
	return this->timestamp;
}

void StreamManager::setTimestamp(long long t) {
    this->timestamp = t;
}

void StreamManager::setSensorTransform(Eigen::Affine3d transform) {

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

  	unsigned long long max_timestamp = Utils::getMaxTimestampForAge(this->max_age);

	// create a comparison object
	StampedPointCloud spc_comp = StampedPointCloud();
  	spc_comp.setTimestamp(Utils::getMaxTimestampForAge(this->max_age));

    // the stream timestamp is the timestamp of the oldest pointcloud of the manager
    this->timestamp = max_timestamp;
	
	// remove all pointclouds not meeting the criteria
	for(auto iter = this->clouds.lower_bound(spc_comp); iter != this->clouds.begin(); iter--) {
		try {
			this->clouds.erase(iter);
		} catch (std::exception &e) {
			std::cout << "Error removing pointcloud: " << e.what() << std::endl;
		}
	}
}
