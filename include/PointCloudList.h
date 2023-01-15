#ifndef POINT_CLOUD_LIST_
#define POINT_CLOUD_LIST_

#include <iostream>
#include <cstring>
#include <cerrno>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointCloudListNode {
	struct PointCloudListNode *next = nullptr;
	struct PointCloudListNode *prev = nullptr;
	pcl::PointCloud<pcl::PointXYZ> *value = nullptr;
};

class PointCloudList {

	private:
		struct PointCloudListNode *head;
		size_t length = 0;

	public:
		PointCloudList();
		~PointCloudList();
		size_t getLength();
		pcl::PointCloud<pcl::PointXYZ> *removeFromHead();
		pcl::PointCloud<pcl::PointXYZ> *remove(pcl::PointCloud<pcl::PointXYZ> *val);
		void insertOnTail(pcl::PointCloud<pcl::PointXYZ> *cloud);
};

#endif
