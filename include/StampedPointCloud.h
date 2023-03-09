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

#define POINTCLOUD_ORIGIN_NONE "NONE"

class StampedPointCloud {

    private:
        long long timestamp;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        bool cloudSet = false;
        bool transformComputed = false;
        std::string originTopic = POINTCLOUD_ORIGIN_NONE;

    public:
        StampedPointCloud();
    
        long long getTimestamp();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloud() const;
        std::string getOriginTopic();

        void setTimestamp(long long t);
        void setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        void setOriginTopic(std::string origin);

        bool isTransformComputed() const;
        void applyTransform(Eigen::Affine3d tf);
};

// custom comparison functor between stamped point clouds
// they are compared by timestamp
struct CompareStampedPointCloud {

    bool operator()(StampedPointCloud first, StampedPointCloud second) const {
        return first.getTimestamp() < second.getTimestamp();
    }
};

#endif