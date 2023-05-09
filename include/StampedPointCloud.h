/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Relate a pointcloud to a timestamp in ms.
* Useful to perform pointcloud aging.
*/

#ifndef STAMPED_POINT_CLOUD_
#define STAMPED_POINT_CLOUD_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <eigen3/Eigen/Dense>
#include <utility>
#include <thread>
#include <mutex>
#include <cstdint>
#include <string>
#include <functional>
#include <cuda_runtime.h>
#include <Utils.h>
#include <cuda_pointclouds.hu>

#define POINTCLOUD_ORIGIN_NONE "NONE"

class StampedPointCloud {

    private:
        unsigned long long timestamp;
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud = nullptr;
        bool cloudSet = false;
        bool transformComputed = false;
        std::string originTopic = POINTCLOUD_ORIGIN_NONE;
        std::uint32_t label; // pointcloud label to allow removal
        std::uint32_t generateLabel();

        std::mutex cloudMutex;

        bool icpTransformComputed = false;

    public:
        StampedPointCloud(const std::string& originTopic);
        ~StampedPointCloud();

        unsigned long long getTimestamp() const;
        pcl::PointCloud<pcl::PointXYZRGBL>::Ptr getPointCloud();
        std::string getOriginTopic() const;
        std::uint32_t getLabel() const;
        bool isIcpTransformComputed() const;

        void setTimestamp(unsigned long long t);
        void setPointCloud(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, bool assignGeneratedLabel=true);
        void setOriginTopic(const std::string& origin);

        bool isTransformComputed() const;
        void applyTransform(const Eigen::Affine3d& tf);

        void applyIcpTransform(const Eigen::Matrix4f& tf);

        void assignLabelToPointCloud(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, std::uint32_t label);

        void removePointsWithLabel(std::uint32_t label);

    friend void transformPointCloudRoutine(StampedPointCloud* instance);
    friend void removePointsWithLabelRoutine(StampedPointCloud* instance, std::uint32_t label);

};

// custom comparison functor between stamped point clouds
// they are compared by timestamp
struct CompareStampedPointCloudPointers {

    bool operator()(const std::shared_ptr<StampedPointCloud>& first, const std::shared_ptr<StampedPointCloud>& second) const {
        return first->getTimestamp() < second->getTimestamp();
    }
};

#endif