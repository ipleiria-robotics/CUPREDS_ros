#include "RGBDDeprojector.h"

// this function is called once from the rgbd_deprojector_node
// it treats everything device related and calls the kernel on the points
__global__ void cuda_deproject_frame(uint8_t *frame, uint16_t width, uint16_t height, float fx, float fy, float cx, float cy, Eigen::Vector3f *cloud) {

    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    
    if(x < width && y < height) {
        // get the pixel depth
        size_t index = y * width + x;
        float depth = frame[index];

        // deproject the pixel to a point
        cloud[0][0] = (x - cx) * depth / fx;
        cloud[0][1] = (y - cy) * depth / fy;
        cloud[0][2] = depth;
    }
}

RGBDDeprojector::RGBDDeprojector() {
    this->K = Eigen::Matrix3f::Identity();
    this->points = NULL;
}

RGBDDeprojector::~RGBDDeprojector() {
    // on instance destruction, free all allocated pointers to avoid memory leaks
    this->freePoints();
    this->freeDevicePoints();
    this->freeDeviceFrame();
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

bool RGBDDeprojector::allocDevicePointsIfNotAllocated(size_t size) {
    if(this->d_points == NULL) {
        cudaError_t err;
        if((err = cudaMalloc(&this->d_points, size * sizeof(Eigen::Vector3f))) != cudaSuccess) {
            ROS_ERROR("Error allocating memory for the pointcloud on the device: %s", cudaGetErrorString(err));
            return false;
        }
    }
    return true;
}

bool RGBDDeprojector::allocDeviceFrameIfNotAllocated(size_t size) {
    if(this->d_frame == NULL) {
        cudaError_t err;
        if((err = cudaMalloc(&this->d_frame, size)) != cudaSuccess) { // each pixel is 1 byte
            ROS_ERROR("Error allocating memory for the frame data on the device: %s", cudaGetErrorString(err));
            return false;
        }
    }
    return true;
}

void RGBDDeprojector::freeDevicePoints() {
    if(this->d_points != NULL) {
        cudaError_t err;
        if((err = cudaFree(&this->d_points)) != cudaSuccess) {
            ROS_ERROR("Error freeing the pointcloud on the device: %s", cudaGetErrorString(err));
        }
    }
}

void RGBDDeprojector::freeDeviceFrame() {
    if(this->d_frame != NULL) {
        cudaError_t err;
        if((err = cudaFree(&this->d_frame)) != cudaSuccess) {
            ROS_ERROR("Error freeing the frame data on the device: %s", cudaGetErrorString(err));
        }
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

    // allocate the device buffers
    if(!this->allocDevicePointsIfNotAllocated(msg->width * msg->height)) {
        return;
    }
    if(!this->allocDeviceFrameIfNotAllocated(msg->width * msg->height)) {
        return;
    }

    // copy the frame data to the device
    cudaError_t err;
    if((err = cudaMemcpy(this->d_frame, &msg->data, num_pixels, cudaMemcpyHostToDevice)) != cudaSuccess) {
        ROS_ERROR("Error copying the frame data to the device: %s", cudaGetErrorString(err));
        return;
    }

    // allocate the pointcloud on the device
    Eigen::Vector3f* d_points;
    if((err = cudaMalloc(&d_points, num_pixels * sizeof(Eigen::Vector3f))) != cudaSuccess) {
        ROS_ERROR("Error allocating memory for the pointcloud on the device: %s", cudaGetErrorString(err));
        return;
    }

    // call the deprojection CUDA kernel
    // the kernel is defined in rgbd_deprojection.cu
    size_t n_blocks = ceil((num_pixels + 1024 - 1) / BLOCK_SIZE);
    cuda_deproject_frame<<<n_blocks, BLOCK_SIZE>>>((uint8_t*) &msg->data, msg->width, msg->height, this->K(0, 0), this->K(1, 1), this->K(0, 2), this->K(1, 2), points);

	// wait for the device to finish
    if((err = cudaDeviceSynchronize()) != cudaSuccess) {
        ROS_ERROR("Error waiting for the device to finish: %s", cudaGetErrorString(err));
        return;
    }

	// copy the deprojected points from the device memory
    if((err = cudaMemcpy(this->points, &d_points, num_pixels * sizeof(Eigen::Vector3f), cudaMemcpyDeviceToHost)) != cudaSuccess) {
        ROS_ERROR("Error copying the deprojected points from the device: %s", cudaGetErrorString(err));
        return;
    }

    // get the deprojection end time
    if(gettimeofday(&end, NULL)) {
        ROS_ERROR("Error getting deprojection end time: %s", strerror(errno));
        return;
    }

    // compute the total deprojection duration in ms
    float deprojection_duration = (end.tv_usec - start.tv_usec) / 1000.0;

    ROS_INFO("Deprojection duration: %f ms", deprojection_duration);

    if(this->point_cloud_pub == nullptr) {
        ROS_WARN("Point cloud publisher not initialized, skipping point cloud publishing");
        return;
    }

    // publish the deprojected points
    sensor_msgs::PointCloud2 point_cloud;
    
}



