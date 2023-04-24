/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the different PointCloud managers and lists, keeping the latest and removing the ones
* older than the max defined age.
*/

#include "PointCloudsManager.h"

#include <utility>

#define GLOBAL_ICP_MAX_CORRESPONDENCE_DISTANCE 1
#define GLOBAL_ICP_MAX_ITERATIONS 1

PointCloudsManager::PointCloudsManager(size_t n_sources, double max_age) {
		this->n_sources = n_sources;

        // initialize empty merged cloud
        this->mergedCloud = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());

		this->max_age = max_age;
}

PointCloudsManager::~PointCloudsManager() {
    // TODO: delete all instances of StreamManagers on the hashtable and set
}

size_t PointCloudsManager::getNClouds() {
	return this->n_sources;
}

bool PointCloudsManager::appendToMerged(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& input) {

	// align the pointclouds
    if(!input->empty()) {
        if(!this->mergedCloud->empty()) {
            // create an ICP instance
            pcl::IterativeClosestPoint<pcl::PointXYZRGBL, pcl::PointXYZRGBL> icp;
            icp.setInputSource(input);
            icp.setInputTarget(this->mergedCloud); // "input" will align to "merged"

            icp.setMaxCorrespondenceDistance(GLOBAL_ICP_MAX_CORRESPONDENCE_DISTANCE);
            icp.setMaximumIterations(GLOBAL_ICP_MAX_ITERATIONS);

            icp.align(*this->mergedCloud); // combine the aligned pointclouds on the "merged" instance

            if (!icp.hasConverged())
                *this->mergedCloud += *input; // if alignment was not possible, just add the pointclouds

            return icp.hasConverged(); // return true if alignment was possible
            
        } else {
            *this->mergedCloud += *input;
        }
        
    }

	return false;
}

void PointCloudsManager::addCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, const std::string& topicName) {

    // the key is not present
    if (this->streamManagers.count(topicName) == 0)
        this->initStreamManager(topicName, this->max_age);

    this->streamManagers[topicName]->addCloud(std::move(cloud));

}

void PointCloudsManager::setTransform(const Eigen::Affine3d& transformEigen, const std::string& topicName) {

	if(this->streamManagers.count(topicName) == 0)
		this->initStreamManager(topicName, this->max_age);
	this->streamManagers[topicName]->setSensorTransform(transformEigen);
}

void PointCloudsManager::initStreamManager(std::string topicName, double max_age) {
    this->streamManagers[topicName] = std::make_unique<StreamManager>(topicName);
    this->streamManagers[topicName]->setMaxAge(max_age);
}

void PointCloudsManager::clearMergedCloud() {
	if(this->mergedCloud != nullptr) {
		this->mergedCloud->clear();
	}
	this->mergedCloudDownsampled = false;
}

void PointCloudsManager::downsampleMergedCloud() {
	
	// create a downsampler instance
	pcl::VoxelGrid<pcl::PointXYZRGBL> downsampler;
	downsampler.setInputCloud(this->mergedCloud);
	downsampler.setLeafSize(FILTER_VOXEL_SIZE, FILTER_VOXEL_SIZE, FILTER_VOXEL_SIZE);
	downsampler.filter(*this->mergedCloud); // replace with the downsampled cloud
}

pcl::PointCloud<pcl::PointXYZRGBL> PointCloudsManager::getMergedCloud() {

	// clear the old merged cloud
	this->clearMergedCloud();

	bool firstCloud = true;

    // iterate the map
    /* TODO: review performance of only perform merging on demand
     * vs merging the pointclouds and removing as needed every time
    */
    for(auto iter = this->streamManagers.begin(); iter != this->streamManagers.end(); ++iter) {
		if(firstCloud) {
			this->mergedCloud = iter->second->getCloud();
			firstCloud = false;
		} else {
			this->appendToMerged(iter->second->getCloud());
		}
    }

 	// this->downsampleMergedCloud();
   	return *this->mergedCloud;
}
