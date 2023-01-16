#ifndef PCL_REGISTRATION_
#define PCL_REGISTRATION_

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "PointCloudsManager.h"

class PCLRegistrator {

    private:
        PointCloudsManager *manager = nullptr;
        size_t n_sources;
        time_t max_pointcloud_age;
        void initializeManager();

    public:
        PCLRegistrator(size_t n_sources, time_t max_age);
        ~PCLRegistrator();
        void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg, std::string topicName);
};

#endif