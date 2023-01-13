#include "RGBDDeprojector.h"

RGBDDeprojector::RGBDDeprojector() {
    this->K = Eigen::Matrix3f::Identity();
    this->points = NULL;
}

RGBDDeprojector::~RGBDDeprojector() {
    // on instance destruction, free all allocated pointers to avoid memory leaks
    this->freePoints();
}

bool RGBDDeprojector::allocPointsIfNotAllocated(size_t size) {
    if(this->points == NULL) {
        this->points = (Eigen::Vector3f *)malloc(size * sizeof(Eigen::Vector3f));
        if(this->points == NULL) {
            ROS_ERROR("Error allocating memory for deprojected points: %s", strerror(errno));
            return false;
        }
    }
    return true;
}

void RGBDDeprojector::freePoints() {
    if(this->points != NULL) {
        free(this->points);
        this->points = NULL;
    }
}

Eigen::Matrix3f RGBDDeprojector::getK() {
    return this->K;
}

void RGBDDeprojector::setK(Eigen::Matrix3f K) {
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
        this->K(row, col) = msg->K[i];
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

    // allocate memory for the deprojected points
    if(!this->allocPointsIfNotAllocated(msg->width * msg->height)) {
        return;
    }

    // clear the point cloud before filling
    this->cloud.clear();
    // deproject the depth image
    for(size_t x = 0; x < msg->width; x++) {
        for(size_t y = 0; y < msg->height; y++) {
            size_t index = y * msg->width + x;
            
            pcl::PointXYZ point;
            point.x = (x - this->K(0, 2)) * msg->data[index] / this->K(0, 0);
            point.y = (y - this->K(1, 2)) * msg->data[index] / this->K(1, 1);
            point.z = msg->data[index];

            // ROS_INFO("Point: %f, %f, %f", point.x, point.y, point.z);

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



