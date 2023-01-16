#ifndef POINTCLOUDS_MANAGER_H_
#define POINTCLOUDS_MANAGER_H_

#include <iostream>
#include <cstring>
#include <cerrno>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "PointCloudList.h"
#include "PCLStamped.h"

// this class manages the stored point clouds
// keep an array of point clouds, the last one for each source
// !!! POINT CLOUDS MUST BE FED HERE ALREADY TRANSFORMED
class PointCloudsManager {

	private:
		size_t n_sources;
		time_t max_age;
		// this array of instances functions almost as a hashtable. details explained on "addCloud"
		PCLStamped **cloudManagers = nullptr; // array of point cloud managers - fixed size = n_clouds
		PointCloudList *clouds = nullptr; // list of the point clouds to consider
		void allocCloudManagers();
		void clean(); // remove clouds older than "maxAge"

	
	public:
		PointCloudsManager(size_t n_sources, time_t max_age);
		~PointCloudsManager();
		size_t getNClouds();
		void addCloud(pcl::PointCloud<pcl::PointXYZ> *cloud, std::string topicName);
		PointCloudList *getClouds();
};

#endif
