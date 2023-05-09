/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* Reference a pointcloud and its capture timestamp on the same class.
* Useful for point cloud management.
*/

#include "StampedPointCloud.h"

StampedPointCloud::StampedPointCloud(const std::string& originTopic) {
    this->timestamp = Utils::getCurrentTimeMillis();

    this->setOriginTopic(originTopic);

    this->label = generateLabel();

    std::lock_guard<std::mutex> lock(this->cloudMutex);
    this->cloud = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBL>());
}

StampedPointCloud::~StampedPointCloud() {
    // free the pointcloud
    std::lock_guard<std::mutex> lock(this->cloudMutex);
    if(this->cloud != nullptr)
        this->cloud.reset();
}

std::uint32_t StampedPointCloud::generateLabel() {

    std::string combined = this->originTopic + std::to_string(this->timestamp);

    std::hash<std::string> hasher;
    std::uint32_t hash_value = hasher(combined);

    return hash_value;
}

unsigned long long StampedPointCloud::getTimestamp() const {
    return this->timestamp;
}

pcl::PointCloud<pcl::PointXYZRGBL>::Ptr StampedPointCloud::getPointCloud() {
    std::lock_guard<std::mutex> lock(this->cloudMutex);
    return this->cloud;
}

std::string StampedPointCloud::getOriginTopic() const {
    return this->originTopic;
}

std::uint32_t StampedPointCloud::getLabel() const {
    return this->label;
}

bool StampedPointCloud::isIcpTransformComputed() const {
    return icpTransformComputed;
}

void StampedPointCloud::setTimestamp(unsigned long long t) {
    this->timestamp = t;
}

void StampedPointCloud::setPointCloud(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& c, bool assignGeneratedLabel) {
    if(c == nullptr)
        return;

    std::lock_guard<std::mutex> lock(this->cloudMutex);
    this->cloud.reset(c.get());

    this->cloudSet = true;

    if(assignGeneratedLabel)
        this->assignLabelToPointCloud(this->cloud, this->label);

}

void StampedPointCloud::assignLabelToPointCloud(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr& cloud, std::uint32_t label) {

    std::lock_guard<std::mutex> lock(this->cloudMutex);
    setPointCloudLabelCuda(cloud, label);
}

void StampedPointCloud::setOriginTopic(const std::string& origin) {
    this->originTopic = origin;
}

bool StampedPointCloud::isTransformComputed() const {
    return this->transformComputed;
}

void StampedPointCloud::applyTransform(const Eigen::Affine3d& tf) {
    // TODO: transform the pointcloud. have in mind they are smart pointers, 
    // attention to performance issues
    std::lock_guard<std::mutex> lock(this->cloudMutex);
    if(this->cloud != nullptr) {

        transformPointCloudCuda(this->cloud, tf);

        // pcl::transformPointCloud(*this->cloud, *this->cloud, tf);
        this->transformComputed = true;
    }
}

void StampedPointCloud::applyIcpTransform(const Eigen::Matrix4f& tf) {

    if(!icpTransformComputed) {

        Eigen::Matrix4d mat4d = tf.cast<double>();
        Eigen::Affine3d affine(mat4d);

        this->applyTransform(affine);

        this->icpTransformComputed = true;
    }
}

void StampedPointCloud::removePointsWithLabel(std::uint32_t label) {

    std::lock_guard<std::mutex> lock(this->cloudMutex);
    if(this->cloud == nullptr)
        return;

    this->cloud->erase(std::remove_if(this->cloud->begin(), this->cloud->end(),
                                     [label](const auto& element) { return element.label == label; }),
                      this->cloud->end());

    // this->cloud.reset();
}
