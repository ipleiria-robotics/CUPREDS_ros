/*
* Copyright (c) 2023 Carlos Tojal.
* All rights reserved.
*
* This node constantly receives the depth and camera info to deal with deprojection.
* It publishes a deprojected point cloud.
*/

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"

void depthInfoCallback(sensor_msgs::CameraInfo::ConstPtr& msg) {

    /* We only use the intrinsics matrix.
    * It doesn't change out of nowhere, only need to store the first message.
    */
}

void depthImageCallback(sensor_msgs::Image::ConstPtr& msg) {

    // Call the deprojection CUDA kernel every time this is called.
}

int main(int argc, char **argv) {

    // TODO: subscribe depth camera info
    // TODO: subscribe depth camera image
    
}
