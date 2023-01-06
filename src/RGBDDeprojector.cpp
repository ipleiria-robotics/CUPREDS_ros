#include "RGBDDeprojector.h"

RGBDDeprojector::RGBDDeprojector() {
    this->K = Eigen::Matrix3f::Identity();
    this->points = NULL;
}

RGBDDeprojector::~RGBDDeprojector() {
    this->freePoints(this->points);
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

void RGBDDeprojector::freePoints(Eigen::Vector3f *points) {
    if(points != NULL) {
        free(points);
        points = NULL;
    }
}

Eigen::Matrix3f RGBDDeprojector::getK() {
    return this->K;
}

void RGBDDeprojector::setK(Eigen::Matrix3f K) {
    this->K = K;
}

void RGBDDeprojector::depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    
    // use openmp threads to copy the intrinsics matrix concurrently
    #pragma omp parallel for
    for(int i = 0; i < 9; i++) {
        int row = i / 3;
        int col = i % 3;
        this->K(row, col) = msg->K[i];
    }
}

void RGBDDeprojector::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    
    struct timeval start, end;

    // get the deprojection start time
    if(gettimeofday(&start, NULL)) {
        ROS_ERROR("Error getting deprojection start time: %s", strerror(errno));
        return;
    }

    // allocate memory for the deprojected points
    if(!this->allocPointsIfNotAllocated(msg->width * msg->height)) {
        return;
    }

    // call the deprojection CUDA kernel
    // the kernel is defined in rgbd_deprojector.cu
    deproject_frame((uint8_t*) &msg->data, msg->width, msg->height, K, points);

    // get the deprojection end time
    if(gettimeofday(&end, NULL)) {
        ROS_ERROR("Error getting deprojection end time: %s", strerror(errno));
        return;
    }

    // compute the total deprojection duration in ms
    float deprojection_duration = (end.tv_usec - start.tv_usec) / 1000.0;

    ROS_INFO("Deprojection duration: %f ms", deprojection_duration);
}



