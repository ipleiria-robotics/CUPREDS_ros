/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*/

#ifndef UTILS_H_
#define UTILS_H_

#include "ros/ros.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>

class Utils {
    public:
        static unsigned long long getCurrentTimeMillis();
        static long getAgeInSecs(unsigned long long timestamp);
        static unsigned long long getMaxTimestampForAge(double age);
        static void removePointCloudFromOther(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud,
                                              const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& pointsToRemove);
};

#endif