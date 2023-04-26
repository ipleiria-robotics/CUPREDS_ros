/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the different stream managers.
*/

#ifndef POINTCLOUDS_MANAGER_H_
#define POINTCLOUDS_MANAGER_H_

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <cstring>
#include <cerrno>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <eigen3/Eigen/Dense>
#include "PointCloudList.h"
#include "StreamManager.h"
#include "Utils.h"
#include <unordered_map>
#include <set>
#include <memory>
#include <mutex>

#define FILTER_VOXEL_SIZE 0.1f

// this class manages the stored point clouds
// keep an array of point clouds, the last one for each source
class PointCloudsManager {

	private:
		size_t n_sources;
		double max_age;
		// the array of instances below functions almost as a hashtable. details explained on "addCloud"
        std::unordered_map<std::string,std::unique_ptr<StreamManager>> streamManagers; // map of stream manager pointers indexed by topic name
		pcl::PointCloud<pcl::PointXYZRGBL>::Ptr mergedCloud; // the merged cloud
		bool mergedCloudDownsampled = false; // prevent double downsampling

		std::mutex managersMutex; // mutex to protect the stream managers
		

		bool appendToMerged(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& input);
		void clearMergedCloud();
		void downsampleMergedCloud();

	
	public:
		PointCloudsManager(size_t n_sources, double max_age);
		~PointCloudsManager();
		size_t getNClouds();
		void addCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, const std::string& topicName);
		void setTransform(const Eigen::Affine3d& transformEigen, const std::string& topicName);
        void initStreamManager(std::string topicName, double max_age);

		pcl::PointCloud<pcl::PointXYZRGBL> getMergedCloud();
};

#endif
