/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the different PointCloud managers and lists, keeping the latest and removing the ones
* older than the max defined age.
*/

#include "PointCloudsManager.h"

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
	for(size_t i = 0; i < this->n_sources; i++) {
		this->cloudManagers[i];
	}
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
	time_t cur_timestamp;
	if((cur_timestamp = time(NULL)) < 0) {
		std::cerr << "Error getting current timestamp: " << strerror(errno) << std::endl;
		return;
	}
	// delete instances older than max_age
	#pragma omp parallel for
	for(size_t i = 0; i < this->n_sources; i++) {
		if(this->cloudManagers[i] != nullptr) {
			if(cur_timestamp - this->cloudManagers[i]->getTimestamp() > max_age)
				this->cloudManagers[i] = nullptr;
		}
	}
}

size_t PointCloudsManager::topicNameToIndex(std::string topicName) {
	// the pointcloud topic names must be "pointcloud0", "pointcloud1", etc.
	// so, we can use the number after "pointcloud" as index on the array
	std::string cloudNumber = topicName.substr(10);
	size_t index = atol(cloudNumber.c_str());

	return index;
}

bool PointCloudsManager::appendToMerged(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {
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

void PointCloudsManager::addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string topicName) {
		

	size_t index = this->topicNameToIndex(topicName);

	// set the pointcloud as the latest of this source
	// check if it was ever defined
	if(this->cloudManagers[index] == nullptr) {
		this->cloudManagers[index] = std::make_shared<StreamManager>();
	} else {
		this->cloudManagers[index]->addCloud(cloud);
	}

	// clean the old pointclouds (defined by max_age)
	// doing this only after insertion avoids instance immediate destruction and recreation upon updating
	this->clean();
}

void PointCloudsManager::setTransform(Eigen::Affine3d transformEigen, std::string topicName) {

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

	for(size_t i = 0; i < this->n_sources; i++) {
		if(this->cloudManagers[i] != nullptr) {
			if(this->cloudManagers[i]->hasCloudReady()) {

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