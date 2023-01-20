/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Manage the PointClouds received by a stream. Keeps the latest PointCloud received,
* registering the timestamp of the update and doing the transformation as set by the tf.
*/

#ifndef STREAM_MANAGER_H
#define STREAM_MANAGER_H

#include "ros/ros.h"
#include <iostream>
#include <cstring>
#include <cerrno>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Dense>

// this class keeps a point cloud with a timestamp
// it is useful to implement the point cloud maximum age
class StreamManager {
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        time_t timestamp = -1; // timestamp of the last update
        Eigen::Affine3d *transform = nullptr;
        bool transformComputed = false; // this is to prevent transforming the cloud multiple times
        void computeTransform();

    public:
        StreamManager();
		~StreamManager();
		
		void addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
		pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
		time_t getTimestamp();
        void setTransform(Eigen::Affine3d *transform);

        bool hasCloudReady();

};

#endif
