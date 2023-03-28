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
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <thread>
#include "StampedPointCloud.h"
#include "Utils.h"

// this class keeps a merged point cloud with a timestamp
// each instance of this class can be seen as the manager for each sensor
// it is useful to implement the point cloud maximum age
class StreamManager {
    private:
        std::string topicName;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = nullptr; // shared pointer to the merged pointcloud
        bool pointCloudSet = false;
        Eigen::Affine3d sensorTransform; // transform of the sensor frame to the robot base
        bool sensorTransformSet = false; // was the sensor transform set?
        Eigen::Affine3d transformToLastState;
        bool lastStateTransformSet;
        void computeTransform();
        std::set<StampedPointCloud, CompareStampedPointCloud> clouds;
        std::queue<StampedPointCloud> clouds_not_transformed;
        double max_age; // max age of the pointclouds in seconds

    public:
        StreamManager(std::string topicName);
		~StreamManager();

        bool operator==(const StreamManager& other) const;

		void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(); // returning the pointer prevents massive memory copies
        void setSensorTransform(Eigen::Affine3d transform);

        void setMaxAge(double max_age);
        double getMaxAge();

        void clear();

};

#endif
