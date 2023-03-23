/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the different PointCloud managers and lists, keeping the latest and removing the ones
* older than the max defined age.
*/

#include "PointCloudsManager.h"

#include <utility>

PointCloudsManager::PointCloudsManager(size_t n_sources, time_t max_age) {
		this->n_sources = n_sources;

        // initialize empty merged cloud
        this->mergedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

		this->max_age = max_age;
}

PointCloudsManager::~PointCloudsManager() {
    // TODO: delete all instances of StreamManagers on the hashtable and set
}

size_t PointCloudsManager::getNClouds() {
	return this->n_sources;
}

// remove pointclouds older than the defined max age
// OBSOLETE
void PointCloudsManager::clean() {
	
    long long max_timestamp = Utils::getMaxTimestampForAge(this->max_age);

	// this "bound" object is used just for search
	std::shared_ptr<StreamManager> bound = std::make_shared<StreamManager>("bound");
	bound->setTimestamp(max_timestamp);

    for(auto iter = this->streamManagers.begin(); iter != this->streamManagers.end(); ++iter)
        iter->second->clear();
}

bool PointCloudsManager::appendToMerged(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input) {
	// PointCloud alignment with ICP is failing, I suppose due to the lack of superposition
	// of the tested dataset. Still to be tested

	// align the pointclouds
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(input);
	icp.setInputTarget(this->mergedCloud); // "input" will align to "merged"
	icp.align(*this->mergedCloud); // combine the aligned pointclouds on the "merged" instance

	if(!icp.hasConverged())
		*mergedCloud += *input; // if alignment was not possible, just add the pointclouds

	return icp.hasConverged(); // return true if alignment was possible
	*this->mergedCloud += *input;

	return false;
}

void PointCloudsManager::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& topicName) {

    // the key is not present
    if (this->streamManagers.count(topicName) == 0) {
        this->streamManagers[topicName] = std::make_shared<StreamManager>(topicName);
    }

    this->streamManagers[topicName]->addCloud(std::move(cloud));

}

void PointCloudsManager::setTransform(const Eigen::Affine3d& transformEigen, const std::string& topicName) {

	if(this->streamManagers.count(topicName) == 0)
		this->streamManagers[topicName] = std::make_shared<StreamManager>(topicName);
	this->streamManagers[topicName]->setSensorTransform(transformEigen);
}

void PointCloudsManager::clearMergedCloud() {
	if(this->mergedCloud != nullptr) {
		this->mergedCloud->clear();
	}
	this->mergedCloudDownsampled = false;
}

void PointCloudsManager::downsampleMergedCloud() {
	
	// create a downsampler instance
	pcl::VoxelGrid<pcl::PointXYZRGB> downsampler;
	downsampler.setInputCloud(this->mergedCloud);
	downsampler.setLeafSize(FILTER_VOXEL_SIZE, FILTER_VOXEL_SIZE, FILTER_VOXEL_SIZE);
	downsampler.filter(*this->mergedCloud); // replace with the downsampled cloud
}

pcl::PointCloud<pcl::PointXYZRGB> PointCloudsManager::getMergedCloud() {

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
