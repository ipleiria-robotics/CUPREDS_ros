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
#include <Utils.h>

#define POINTCLOUD_ORIGIN_NONE "NONE"

class StampedPointCloud {

    private:
        unsigned long long timestamp;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        bool cloudSet = false;
        bool transformComputed = false;
        std::string originTopic = POINTCLOUD_ORIGIN_NONE;

    public:
        StampedPointCloud();

        unsigned long long getTimestamp();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud() const;
        std::string getOriginTopic();

        void setTimestamp(unsigned long long t);
        void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void setOriginTopic(std::string origin);

        bool isTransformComputed() const;
        void applyTransform(Eigen::Affine3d tf);
};

// custom comparison functor between stamped point clouds
// they are compared by timestamp
struct CompareStampedPointCloudPointers {

    bool operator()(std::shared_ptr<StampedPointCloud> first, std::shared_ptr<StampedPointCloud> second) const {
        return first->getTimestamp() < second->getTimestamp();
    }
};

#endif