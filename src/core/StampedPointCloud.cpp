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

void StampedPointCloud::setPointCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr c, bool assignGeneratedLabel) {
    this->cloudSet = true;
    this->cloud = c;

    if(assignGeneratedLabel)
        this->assignLabelToPointCloud(this->cloud, this->label);
}

void StampedPointCloud::assignLabelToPointCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud, std::uint32_t label) {

    setPointCloudLabelCuda(cloud, label);
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

        transformPointCloudCuda(this->cloud, tf);

        // pcl::transformPointCloud(*this->cloud, *this->cloud, tf);
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

void StampedPointCloud::removePointsWithLabel(std::uint32_t label) {

    for(auto it = this->cloud->begin(); it != this->cloud->end(); it++) {
        if(it->label == label) {
            this->cloud->erase(it);
        }
    }
}
