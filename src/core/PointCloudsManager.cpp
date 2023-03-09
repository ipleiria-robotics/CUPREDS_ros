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
	// get the current timestamp to calculate pointclouds age
    long long cur_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

	// remove all the stream managers older than the max age with O(logN + k) time complexity
	long long min_timestamp = cur_timestamp - (this->max_age * 1000);
	// this "bound" object is used just for search
	std::shared_ptr<StreamManager> bound = std::make_shared<StreamManager>("bound");
	bound->setTimestamp(cur_timestamp - (this->max_age * 1000));

	// find the last element with smaller order than the criteria
	auto iter = this->streamsToMerge.lower_bound(bound);

	// remove all previous elements
	while(iter != this->streamsToMerge.begin()) {
		--iter;
		this->streamsToMerge.erase(iter);
	}
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

    // the key is not present
    if (this->streamManagers.count(topicName) == 0) {
        this->streamManagers[topicName] = std::make_shared<StreamManager>(topicName);
    }

    this->streamManagers[topicName]->addCloud(std::move(cloud));

	/*
	// remove the stream manager from the set, due to the reordering
	this->streamsToMerge.erase(this->streamManagers[topicName]);

	// reinsert the stream manager on the set
	this->streamsToMerge.insert(this->streamManagers[topicName]);
	*/
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

	// TODO: review performance of only perform merging on demand
	// points from pointclouds older than the max age can be removed with "ExtractIndices"
	for(auto iter = this->streamsToMerge.begin(); iter != this->streamsToMerge.end(); ++iter) {
		if((*iter)->hasCloudReady()) {

			if(firstCloud) {
				*this->mergedCloud += *(*iter)->getCloud();
			} else {
				this->appendToMerged((*iter)->getCloud());
			}
		}
		iter++;
	}

	this->downsampleMergedCloud();

	return *this->mergedCloud;
}
