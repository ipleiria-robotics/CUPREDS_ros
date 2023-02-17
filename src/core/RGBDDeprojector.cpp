/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Deproject depth images to point clouds.
*/

#include "RGBDDeprojector.h"

RGBDDeprojector::RGBDDeprojector() {
    this->K = Eigen::Matrix3d::Identity();
    this->colorImageSet = false;
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

void RGBDDeprojector::setPointCloudPublisher(std::shared_ptr<ros::Publisher> point_cloud_pub) {
    this->point_cloud_pub = point_cloud_pub;
}

void RGBDDeprojector::depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    
    // use openmp threads to copy the intrinsics matrix concurrently
	// loop unrolling is used to increase performance
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

	// convert the sensor image to an opencv matrix
	cv_bridge::CvImagePtr cv_ptr;
	try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
	} catch(cv_bridge::Exception& e) {
			ROS_ERROR("Error converting the sensor image into an OpenCV matrix: %s", e.what());
			return;
	}
	cv::Mat depth_image = cv_ptr->image;

	// convert the eigen intrinsic matrix to an opencv matrix
	cv::Mat K_cv;
	cv::eigen2cv(this->K, K_cv);	

    // clear the point cloud before filling
    this->cloud.clear();

	// use opencv to reproject the points from the depth image and intrinsics to a cv mat
	cv::Mat points_cv;
	cv::reprojectImageTo3D(depth_image, points_cv, K_cv);

	// convert the opencv matrix into a pcl pointcloud
	this->cloud.width = points_cv.cols;
	this->cloud.height = points_cv.rows;
	this->cloud.points.resize(points_cv.total());
	
	// TODO: this is EXTREMELY processor intensive and slow, should do with CUDA instead
	// at least use some kind of thread pool
	for(int i = 0; i < points_cv.rows; i++) {
			for(int j = 0; j < points_cv.cols; j++) {
					pcl::PointXYZRGB& point = this->cloud.points[i * points_cv.cols + j];
					cv::Vec3f& vec = points_cv.at<cv::Vec3f>(i,j);

					point.x = vec[0];
					point.y = vec[1];
					point.z = vec[2];
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
    pcl::toROSMsg(this->cloud, point_cloud); // create a ros message from the pointcloud
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
    this->last_color_image = cv_ptr->image;
    this->colorImageSet = true;
}
