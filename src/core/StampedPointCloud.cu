/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Reference a pointcloud and its capture timestamp on the same class.
* Useful for point cloud management.
*/

#include "StampedPointCloud.h"

StampedPointCloud::StampedPointCloud(std::string originTopic) {
    this->timestamp = Utils::getCurrentTimeMillis();

    this->setOriginTopic(originTopic);

    this->label = generateLabel();
}

std::uint32_t StampedPointCloud::generateLabel() {

    std::string combined = this->originTopic + std::to_string(this->timestamp);

    std::hash<std::string> hasher;
    std::uint32_t hash_value = hasher(combined);

    return hash_value;
}

unsigned long long StampedPointCloud::getTimestamp() {
    return this->timestamp;
}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr StampedPointCloud::getPointCloud() const {
    return this->cloud;
}

std::string StampedPointCloud::getOriginTopic() {
    return this->originTopic;
}

std::uint32_t StampedPointCloud::getLabel() {
    return this->label;
}

bool StampedPointCloud::isIcpTransformComputed() {
    return icpTransformComputed;
}

void StampedPointCloud::setTimestamp(unsigned long long t) {
    this->timestamp = t;
}

void StampedPointCloud::setPointCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr c) {
    this->cloudSet = true;
    this->cloud = c;

    this->assignLabelToPointCloud(this->cloud, this->label);
}

void StampedPointCloud::assignLabelToPointCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, std::uint32_t label) {
    cudaError_t err = cudaSuccess;

    // declare the device input point array
    pcl::PointXYZRGBL *d_cloud;

    // allocate memory on the device to store the input pointcloud
    if((err = cudaMalloc(&d_cloud, cloud->size() * sizeof(pcl::PointXYZRGBL))) != cudaSuccess) {
        std::cerr << "Error allocating memory for the pointcloud: " << cudaGetErrorString(err) << std::endl;
        return;
    }

    // copy the input pointcloud to the device
    if((err = cudaMemcpy(d_cloud, cloud->points.data(), cloud->size() * sizeof(pcl::PointXYZRGBL), cudaMemcpyHostToDevice)) != cudaSuccess) {
        std::cerr << "Error copying the input pointcloud to the device: " << cudaGetErrorString(err) << std::endl;
        return;
    }

    // call the kernel
    dim3 block(512);
    dim3 grid((cloud->size() + block.x - 1) / block.x);
    setPointLabelKernel<<<grid,block>>>(d_cloud, label, cloud->size());

    // copy the output pointcloud back to the host
    if((err = cudaMemcpy(cloud->points.data(), d_cloud, cloud->size() * sizeof(pcl::PointXYZRGBL), cudaMemcpyDeviceToHost)) != cudaSuccess) {
        std::cerr << "Error copying the output pointcloud to the host: " << cudaGetErrorString(err) << std::endl;
        return;
    }
}

void StampedPointCloud::setOriginTopic(std::string origin) {
    this->originTopic = origin;
}

bool StampedPointCloud::isTransformComputed() const {
    return this->transformComputed;
}

void StampedPointCloud::applyTransform(Eigen::Affine3d tf) {
    // TODO: transform the pointcloud. have in mind they are smart pointers, 
    // attention to performance issues
    if(this->cloudSet) {
        pcl::transformPointCloud(*this->cloud, *this->cloud, tf);
        this->transformComputed = true;
    }
}

void StampedPointCloud::applyIcpTransform(Eigen::Matrix4f tf) {

    if(!icpTransformComputed) {

        Eigen::Matrix4d mat4d = tf.cast<double>();
        Eigen::Affine3d affine(mat4d);

        this->applyTransform(affine);

        this->icpTransformComputed = true;
    }
}
