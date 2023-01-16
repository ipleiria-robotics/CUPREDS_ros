#ifndef STREAM_MANAGER_H
#define STREAM_MANAGER_H

#include <iostream>
#include <cstring>
#include <cerrno>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// this class keeps a point cloud with a timestamp
// it is useful to implement the point cloud maximum age
class StreamManager {
    private:
        pcl::PointCloud<pcl::PointXYZ> *cloud;
        time_t timestamp;
    public:
        StreamManager(pcl::PointCloud<pcl::PointXYZ> *cloud);
		~StreamManager();
		
		void addCloud(pcl::PointCloud<pcl::PointXYZ> *cloud);
		pcl::PointCloud<pcl::PointXYZ> *getCloud();
		time_t getTimestamp();
};

#endif
