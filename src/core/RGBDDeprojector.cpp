/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Deproject depth images to point clouds.
*/

#include "RGBDDeprojector.h"

RGBDDeprojector::RGBDDeprojector() {
    this->K = Eigen::Matrix3d::Identity();
}

RGBDDeprojector::~RGBDDeprojector() {

}

Eigen::Matrix<double, 3, 3> RGBDDeprojector::getK() {
    return this->K;
}

void RGBDDeprojector::setK(Eigen::Matrix<double, 3, 3> K) {
    ROS_INFO("K updated");
    this->K = K;
}

ros::Publisher RGBDDeprojector::getPointCloudPublisher() {
    return *this->point_cloud_pub;
}

void RGBDDeprojector::setPointCloudPublisher(ros::Publisher *point_cloud_pub) {
    this->point_cloud_pub = point_cloud_pub;
}

void RGBDDeprojector::depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    
    // use openmp threads to copy the intrinsics matrix concurrently
    #pragma omp parallel for
    for(int i = 0; i < 9; i++) {
        int row = i / 3;
        int col = i % 3;
        this->K(row, col) = (double) msg->K[i];
    }

    // get the camera frame id
    this->camera_frame_id = msg->header.frame_id;
}

void RGBDDeprojector::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    
    struct timeval start, end;

    size_t num_pixels = msg->width * msg->height;

    // get the deprojection start time
    if(gettimeofday(&start, NULL)) {
        ROS_ERROR("Error getting deprojection start time: %s", strerror(errno));
        return;
    }

    // clear the point cloud before filling
    this->cloud.clear();
    // deproject the depth image
    for(size_t x = 0; x < msg->width; x++) {
        for(size_t y = 0; y < msg->height; y++) {
            size_t index = y * msg->width + x;

            /*
            Eigen::Matrix<double, 3, 1> pixel_vector = {
                (double) x,
                (double) y,
                1.0
            };

            Eigen::Matrix<double, 3, 1> point_vector = (this->K * pixel_vector) / msg->data[index];
            */
            
            pcl::PointXYZRGB point;
            double z = (double) msg->data[index] * 50 / 256;
            // deproject the point position
            point.x = (x - this->K(0, 2)) * z / this->K(0, 0);
            point.y = (y - this->K(1, 2)) * z / this->K(1, 1);
            point.z = z;
            /*
            point.x = point_vector(0);
            point.y = point_vector(1);
            point.z = point_vector(2);
            */

            // check if a color frame is present
            if(this->last_color_image != nullptr) {
                // fill the point color
                // 8 bits for each channel (0-255), 3 channels (BGR)
                /*
                point.rgb = this->last_color_image->at<cv::Vec3b>(y, x)[2] << 16 |
                            this->last_color_image->at<cv::Vec3b>(y, x)[1] << 8 |
                            this->last_color_image->at<cv::Vec3b>(y, x)[0];
                            */
                
            }

            this->cloud.push_back(point);
        }
    }

    // get the deprojection end time
    if(gettimeofday(&end, NULL)) {
        ROS_ERROR("Error getting deprojection end time: %s", strerror(errno));
        return;
    }

    // compute the total deprojection duration in ms
    float deprojection_duration = (end.tv_usec - start.tv_usec) / 1000.0;

    // ROS_INFO("Deprojection duration: %f ms", deprojection_duration);

    // check if a publisher was given
    if(this->point_cloud_pub == nullptr) {
        ROS_WARN("Point cloud publisher not initialized, skipping point cloud publishing");
        return;
    }

    // publish the deprojected points
    sensor_msgs::PointCloud2 point_cloud;
    pcl::toROSMsg(this->cloud, point_cloud);
    point_cloud.header.frame_id = this->camera_frame_id;
    this->point_cloud_pub->publish(point_cloud);
}

void RGBDDeprojector::colorImageCallback(const sensor_msgs::Image::ConstPtr& msg) {

    // store the last received color image as a OpenCV matrix
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Error converting color image to OpenCV matrix: %s", e.what());
        return;
    }
    this->last_color_image = &(cv_ptr->image);
}
