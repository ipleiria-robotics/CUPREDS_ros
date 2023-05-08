/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the PointClouds received by a stream. Keeps the latest PointCloud received,
* registering the timestamp of the update and doing the transformation as set by the tf.
*/

#include "StreamManager.h"

#include <utility>

#define STREAM_ICP_MAX_CORRESPONDENCE_DISTANCE 1
#define STREAM_ICP_MAX_ITERATIONS 10

StreamManager::StreamManager(std::string topicName) {
    this->topicName = topicName;
    this->cloud = std::make_shared<StampedPointCloud>(topicName);
}

StreamManager::~StreamManager() {
    this->cloud.reset();
}

bool StreamManager::operator==(const StreamManager &other) const {
    return this->topicName == other.topicName;
}

void StreamManager::removePointCloud(std::shared_ptr<StampedPointCloud> spcl) {

    // remove points with that label from the merged pointcloud
    this->cloud->removePointsWithLabel(spcl->getLabel());

    // lock the set
    std::lock_guard<std::mutex> guard(this->setMutex);

    // iterate the set
    for (auto it = this->clouds.begin(); it != this->clouds.end();) {
        if ((*it)->getLabel() == spcl->getLabel()) {
            // remove the pointcloud from the set
            it = this->clouds.erase(it);
        } else {
            ++it;
        }
    }

    spcl.reset();
}

void StreamManager::computeTransform() {

	while(!this->clouds_not_transformed.empty()) {
		
		// get the first element
		std::shared_ptr<StampedPointCloud> spcl = this->clouds_not_transformed.front();
		spcl->applyTransform(this->sensorTransform);

		// add to the set
		this->clouds.insert(spcl);

        this->clouds_not_transformed.front().reset();
		// remove from the queue
		this->clouds_not_transformed.pop();

        spcl.reset();
	}

	this->pointCloudSet = true;
}

// this is a routine to call from a thread to transform a pointcloud
void applyTransformRoutine(StreamManager *instance, std::shared_ptr<StampedPointCloud> spcl, Eigen::Affine3d tf) {
	spcl->applyTransform(tf);
    spcl.reset();
}

// this is a routine to call from a thread to clear the pointclouds which don't meet the criteria
void clearPointCloudsRoutine(StreamManager *instance) {
	instance->clear();
}

// this routine is called in a thread
// this allow to automatically remove the pointclouds which are older than the max age
// as well as the removal is lazy, so it doesn't lock the main thread
void pointCloudAutoRemoveRoutine(StreamManager* instance, std::shared_ptr<StampedPointCloud> spcl) {

    // sleep for the max age
    std::this_thread::sleep_for(std::chrono::milliseconds(
            static_cast<long long>(instance->max_age * 1000)));

    // call the pointcloud removal method
    instance->removePointCloud(std::move(spcl));
}

void icpTransformPointCloudRoutine(std::shared_ptr<StampedPointCloud> spcl, Eigen::Matrix4f tf) {

    spcl->applyIcpTransform(tf);
    spcl.reset();
}

void StreamManager::addCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud) {

    if(cloud == nullptr)
        return;
    if(cloud->empty()) {
        cloud.reset();
        return;
    }

	// create a stamped point cloud object to keep this pointcloud
   	std::shared_ptr<StampedPointCloud> spcl = std::make_shared<StampedPointCloud>(this->topicName);
	spcl->setPointCloud(std::move(cloud));

	if(this->sensorTransformSet) {
		// transform the incoming pointcloud and add directly to the set

		// start a thread to transform the pointcloud
		std::thread transformationThread(applyTransformRoutine, this, spcl, sensorTransform);

		// start a thread to clear the pointclouds older than max age
		// std::thread cleaningThread(clearPointCloudsRoutine, this);

		// wait for both threads to synchronize
		transformationThread.join();
		// cleaningThread.join();

		// add the new pointcloud to the set
        /*
        int startingIndex = this->clouds.size();
		this->clouds.insert(spcl);
        int endingIndex = this->clouds.size();

        // store the indices in the merged cloud
        for(int i = startingIndex; i < endingIndex; i++) {
            spcl->addMergedIndex(i);
        }*/

        try {
            if(!spcl->getPointCloud()->empty()) {
                this->cloudMutex.lock();
                if(!this->cloud->getPointCloud()->empty()) {
                    /*
                    pcl::IterativeClosestPoint<pcl::PointXYZRGBL,pcl::PointXYZRGBL> icp;

                    icp.setInputSource(spcl->getPointCloud());
                    icp.setInputTarget(this->cloud->getPointCloud());

                    icp.setMaxCorrespondenceDistance(STREAM_ICP_MAX_CORRESPONDENCE_DISTANCE);
                    icp.setMaximumIterations(STREAM_ICP_MAX_ITERATIONS);

                    icp.align(*this->cloud->getPointCloud());

                    if (!icp.hasConverged()) {
                        *this->cloud->getPointCloud() += *spcl->getPointCloud(); // if alignment was not possible, just add the pointclouds
                        
                    }*/

                    *this->cloud->getPointCloud() = *spcl->getPointCloud();

                } else {
                    *this->cloud->getPointCloud() = *spcl->getPointCloud();
                }

                // remove the points. they are not needed, just the label
                spcl->getPointCloud().reset();

                this->cloudMutex.unlock();

                // start the pointcloud recycling thread
                std::thread spclRecyclingThread(pointCloudAutoRemoveRoutine, this, spcl);
                // detach from the thread, this execution flow doesn't really care about it
                spclRecyclingThread.detach();
            }

        } catch (std::exception &e) {
            std::cout << "Error performing sensor-wise ICP: " << e.what() << std::endl;
        }

	} else {

		// add the pointcloud to the queue
		this->clouds_not_transformed.push(spcl);
	}

    spcl.reset();

}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr StreamManager::getCloud() {
	return this->cloud->getPointCloud();
}

void StreamManager::setSensorTransform(Eigen::Affine3d transform) {

    std::lock_guard<std::mutex> lock(this->sensorTransformMutex);
    
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

    // CURRENTLY DEPRECATED: pointclouds are currently autoremoved

  	unsigned long long max_timestamp = Utils::getMaxTimestampForAge(this->max_age);

	// create a comparison object
	std::shared_ptr<StampedPointCloud> spc_comp = std::make_shared<StampedPointCloud>(this->topicName);
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
