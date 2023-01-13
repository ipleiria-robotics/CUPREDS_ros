#ifndef RGBD_DEPROJECTOR_H
#define RGBD_DEPROJECTOR_H

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <eigen3/Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cstring>
#include <cerrno>
#include <sys/time.h>

class RGBDDeprojector {

    private:
        Eigen::Matrix3f K;
        std::string camera_frame_id;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        Eigen::Vector3f *points = NULL;
        ros::Publisher *point_cloud_pub = nullptr;

    public:
        RGBDDeprojector();
        ~RGBDDeprojector();

        // host pointcloud allocation and deallocation
        bool allocPointsIfNotAllocated(size_t size);
        void freePoints();

        Eigen::Matrix3f getK();
        void setK(Eigen::Matrix3f K);

        ros::Publisher getPointCloudPublisher();
        void setPointCloudPublisher(ros::Publisher *point_cloud_pub);

        void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
        void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);
};

#endif