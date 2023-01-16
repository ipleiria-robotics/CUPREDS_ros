/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the different PointCloud managers and lists, keeping the latest and removing the ones
* older than the max defined age.
*/

#ifndef POINTCLOUDS_MANAGER_H_
#define POINTCLOUDS_MANAGER_H_

#include <iostream>
#include <cstring>
#include <cerrno>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <eigen3/Eigen/Dense>
#include "PointCloudList.h"
#include "StreamManager.h"

// this class manages the stored point clouds
// keep an array of point clouds, the last one for each source
class PointCloudsManager {

	private:
		size_t n_sources;
		time_t max_age;
		// the array of instances below functions almost as a hashtable. details explained on "addCloud"
		StreamManager **cloudManagers = nullptr; // array of point cloud managers - fixed size = n_clouds
		PointCloudList *clouds = nullptr; // list of the point clouds to consider
		pcl::PointCloud<pcl::PointXYZ>::Ptr mergedCloud = nullptr; // the merged cloud
		bool mergedCloudDownsampled = false; // prevent double downsampling
		void allocCloudManagers();
		void clean(); // remove clouds older than "maxAge"
		size_t topicNameToIndex(std::string topicName);

		bool appendToMerged(pcl::PointCloud<pcl::PointXYZ>::Ptr input);
		void downsampleMergedCloud(); // TODO

	
	public:
		PointCloudsManager(size_t n_sources, time_t max_age);
		~PointCloudsManager();
		size_t getNClouds();
		void addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string topicName);
		void setTransform(Eigen::Affine3d *transformEigen, std::string topicName);
		PointCloudList *getClouds();

		pcl::PointCloud<pcl::PointXYZ>::Ptr getMergedCloud();
};

#endif
