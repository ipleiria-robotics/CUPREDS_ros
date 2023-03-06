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
#include <eigen3/Eigen/Dense>
#include <memory>
#include <thread>
#include "StampedPointCloud.h"

// this class keeps a merged point cloud with a timestamp
// each instance of this class can be seen as the manager for each sensor
// it is useful to implement the point cloud maximum age
class StreamManager {
    private:
        std::string topicName;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; // shared pointer to the merged pointcloud
        long long timestamp = -1; // timestamp of the last update in milliseconds
        Eigen::Affine3d sensorTransform; // transform of the sensor frame to the robot base
        bool sensorTransformSet = false; // was the sensor transform set?
        bool sensorTransformComputed = false; // this is to prevent transforming the cloud multiple times
        Eigen::Affine3d transformToLastState;
        bool lastStateTransformSet;
        void computeTransform();
        std::set<StampedPointCloud, CompareStampedPointCloud> clouds;
        time_t max_age;

    public:
        StreamManager(std::string topicName);
		~StreamManager();

        bool operator<(const StreamManager& other) const;
        bool operator>(const StreamManager& other) const;
        bool operator==(const StreamManager& other) const;
		
        void clearCloud(); // clear the sensor's merged pointcloud
		void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(); // returning the pointer prevents massive memory copies
		time_t getTimestamp();
        void setTimestamp(long long timestamp);
        void setSensorTransform(Eigen::Affine3d transform);

        void setMaxAge(time_t max_age);
        time_t getMaxAge();

        void clear(); // clear the stream's pointcloud

        bool hasCloudReady();

};

// custom comparison functor between shared pointers of StreamManagers
// the comparison is made by timestamp
struct CompareStreamManager {

    bool operator()(const std::shared_ptr<StreamManager>& first, const std::shared_ptr<StreamManager>& second) const {
        return first->getTimestamp() < second->getTimestamp();
    }
};

#endif
