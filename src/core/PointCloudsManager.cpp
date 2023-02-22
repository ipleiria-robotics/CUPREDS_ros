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

		// allocate the array
		this->allocCloudManagers();

        // initialize empty merged cloud
        this->mergedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

		this->max_age = max_age;
}

PointCloudsManager::~PointCloudsManager() {
	// delete each of the instances first
	#pragma omp parallel for
	for(size_t i = 0; i < this->n_sources; i++)
		this->cloudManagers[i].reset();
}

size_t PointCloudsManager::getNClouds() {
	return this->n_sources;
}

void PointCloudsManager::allocCloudManagers() {
	// initialize all instances to nullptr
	# pragma omp parallel for
	for(size_t i = 0; i < this->n_sources; i++)
		this->cloudManagers.push_back(nullptr);
}

// remove pointclouds older than the defined max age
void PointCloudsManager::clean() {
	// get the current timestamp to calculate pointclouds age
    long long cur_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

	// delete instances older than max_age
	#pragma omp parallel for
	for(size_t i = 0; i < this->n_sources; i++) {
		if(this->cloudManagers[i] != nullptr) {
            if (this->cloudManagers[i]->getTimestamp() < cur_timestamp - (this->max_age * 1000)) {
                this->cloudManagers[i].reset();
                this->cloudManagers[i] = nullptr;
            }
        }
	}
}

size_t PointCloudsManager::topicNameToIndex(const std::string& topicName) {
	// the pointcloud topic names must be "pointcloud0", "pointcloud1", etc.
	// so, we can use the number after "pointcloud" as index on the array
	std::string cloudNumber = topicName.substr(10);
	size_t index = strtol(cloudNumber.c_str(), nullptr, 10);

	return index;
}

bool PointCloudsManager::appendToMerged(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input) {
	// PointCloud alignment with ICP is failing, I suppose due to the lack of superposition
	// of the tested dataset. Still to be tested
	/*
	// align the pointclouds
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(input);
	icp.setInputTarget(this->mergedCloud); // "input" will align to "merged"
	icp.align(*this->mergedCloud); // combine the aligned pointclouds on the "merged" instance

	if(!icp.hasConverged())
		*mergedCloud += *input; // if alignment was not possible, just add the pointclouds

	return icp.hasConverged(); // return true if alignment was possible
	*/
	*this->mergedCloud += *input;

	return false;
}

void PointCloudsManager::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& topicName) {

	size_t index = this->topicNameToIndex(topicName);

	// check if it was ever defined
	if(this->cloudManagers[index] == nullptr) {
		this->cloudManagers[index] = std::make_shared<StreamManager>();
	} else {
        // add the new cloud to the corresponding stream manager
		this->cloudManagers[index]->addCloud(std::move(cloud));
	}

	// clean the old pointclouds (defined by max_age)
	// doing this only after insertion avoids instance immediate destruction and recreation upon updating
	// this->clean();
}

void PointCloudsManager::setTransform(const Eigen::Affine3d& transformEigen, const std::string& topicName) {

	size_t index = this->topicNameToIndex(topicName);

	// check if it was ever defined
	if(this->cloudManagers[index] == nullptr) {
		this->cloudManagers[index] = std::make_shared<StreamManager>();
	} else {
		this->cloudManagers[index]->setTransform(transformEigen);
	}
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

    long long cur_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

    #pragma omp parallel for
	for(size_t i = 0; i < this->n_sources; i++) {
		if(this->cloudManagers[i] != nullptr) {
			if(this->cloudManagers[i]->hasCloudReady() && this->cloudManagers[i]->getTimestamp() >= cur_timestamp - (this->max_age * 1000)) {

				// on the first pointcloud, the merged version is itself
				if(firstCloud) {
					*this->mergedCloud += *this->cloudManagers[i]->getCloud();
					firstCloud = false;
				} else {
					this->appendToMerged(this->cloudManagers[i]->getCloud());
				}
			}
		}
	}

	this->downsampleMergedCloud();

	return *this->mergedCloud;
}