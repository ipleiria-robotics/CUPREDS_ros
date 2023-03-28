/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the PointClouds received by a stream. Keeps the latest PointCloud received,
* registering the timestamp of the update and doing the transformation as set by the tf.
*/

#include "StreamManager.h"

#include <utility>

#define STREAM_ICP_MAX_CORRESPONDENCE_DISTANCE 0.1
#define STREAM_ICP_MAX_ITERATIONS 10

StreamManager::StreamManager(std::string topicName) {
    this->topicName = topicName;
	this->cloud = pcl::PointCloud<pcl::PointXYZRGB>().makeShared();
}

StreamManager::~StreamManager() {
    this->cloud.reset();
}

bool StreamManager::operator==(const StreamManager &other) const {
    return this->topicName == other.topicName;
}

void StreamManager::removePointCloud(std::shared_ptr<StampedPointCloud> spcl) {

    // guard access to the set
    std::lock_guard<std::mutex> guard(this->setMutex);

    // remove from the set
    this->clouds.erase(spcl);

    // TODO: remove from the mergedCloud

    // force the pointer deletion
    spcl.reset();
}

void StreamManager::computeTransform() {

	while(this->clouds_not_transformed.size() > 0) {
		
		// get the first element
		std::shared_ptr<StampedPointCloud> spcl = this->clouds_not_transformed.front();
		spcl->applyTransform(this->sensorTransform);

		// add to the set
		this->clouds.insert(spcl);

		// remove from the queue
		this->clouds_not_transformed.pop();
	}

	this->pointCloudSet = true;
}

// this is a routine to call from a thread to transform a pointcloud
void applyTransformRoutine(StreamManager *instance, std::shared_ptr<StampedPointCloud> spcl, Eigen::Affine3d tf) {
    std::lock_guard<std::mutex> guard(instance->setMutex);
	spcl->applyTransform(tf);
}

// this is a routine to call from a thread to clear the pointclouds which don't meet the criteria
void clearPointCloudsRoutine(StreamManager *instance) {
	instance->clear();
}

void pointCloudAutoRemoveRoutine(StreamManager* instance, std::shared_ptr<StampedPointCloud> spcl) {

    // sleep for the max age
    std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<long long>(instance->max_age * 1000)));

    // call the pointcloud removal method
    instance->removePointCloud(spcl);
}

void StreamManager::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	// create a stamped point cloud object to keep this pointcloud
  	std::shared_ptr<StampedPointCloud> spcl = std::make_shared<StampedPointCloud>();
	spcl->setOriginTopic(this->topicName);
	spcl->setPointCloud(cloud);

	if(this->sensorTransformSet) {
		// transform the incoming pointcloud and add directly to the set

		// start a thread to transform the pointcloud
		std::thread transformationThread(applyTransformRoutine, this, spcl, sensorTransform);

		// start a thread to clear the pointclouds older than max age
		std::thread cleaningThread(clearPointCloudsRoutine, this);

		// wait for both threads to synchronize
		transformationThread.join();
		cleaningThread.join();

		// add the new pointcloud to the set
		this->clouds.insert(spcl);

	} else {

		// add the pointcloud to the queue
		this->clouds_not_transformed.push(spcl);
	}

    // start the pointcloud recycling thread
    std::thread spclRecyclingThread(pointCloudAutoRemoveRoutine, this, spcl);
    // detach from the thread, this execution flow doesn't really care about it
    spclRecyclingThread.detach();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StreamManager::getCloud() {

	// clear the old cloud
	this->cloud->empty();

	// create a icp object
	pcl::IterativeClosestPoint<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;

	bool firstCloud = true;

    // clear the set
    // this->clear();

    // lock access to the set to iterate it
    std::lock_guard<std::mutex> guard(this->setMutex);

    // iterator
    std::set<std::shared_ptr<StampedPointCloud>,CompareStampedPointCloudPointers>::iterator it;

	for(it = this->clouds.begin(); it != this->clouds.end(); ++it) {
		if(firstCloud) {
			// copy the first pointcloud to the cloud
			pcl::copyPointCloud(*(*it)->getPointCloud(), *this->cloud);
			firstCloud = false;
			continue;
		}
		*this->cloud += *(*it)->getPointCloud();
	}

	// iterate over all pointclouds in the set and do ICP
	/*
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

            icp.setMaxCorrespondenceDistance(STREAM_ICP_MAX_CORRESPONDENCE_DISTANCE);
            icp.setMaximumIterations(STREAM_ICP_MAX_ITERATIONS);

			icp.align(*this->cloud);

		} catch (std::exception &e) {
			std::cout << "Error performing sensor-wise ICP: " << e.what() << std::endl;
		}
	}
	*/

	this->pointCloudSet = true;
	return this->cloud;
}

void StreamManager::setSensorTransform(Eigen::Affine3d transform) {

	// set the new transform
    this->sensorTransform = transform;
    this->sensorTransformSet = true;
	this->computeTransform();
}

void StreamManager::setMaxAge(double max_age) {
	this->max_age = max_age;
}

double StreamManager::getMaxAge() {
	return this->max_age;
}

void StreamManager::clear() {

  	unsigned long long max_timestamp = Utils::getMaxTimestampForAge(this->max_age);

	// create a comparison object
	std::shared_ptr<StampedPointCloud> spc_comp = std::make_shared<StampedPointCloud>();
  	spc_comp->setTimestamp(max_timestamp);

      // lock access to the set
      std::lock_guard<std::mutex> guard(this->setMutex);

      // remove all pointclouds not meeting criteria
    auto lower = this->clouds.lower_bound(spc_comp);
    try {
        this->clouds.erase(this->clouds.begin(), lower);
    } catch (std::exception &e) {
        std::cerr << "Error removing pointcloud: " << e.what() << std::endl;
    }

    // remove the comparison pointcloud
    spc_comp.reset();
}
