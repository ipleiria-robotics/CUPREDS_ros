# SafeForest Point Cloud Aggregator

This package is intended to provide functionality to aggregate pointclouds provenient from different sources. For instance, a deprojected depth camera and LiDAR sensors. The hardware tested during the development of this package was the RealSense D435i depth camera and 3 Livox Horizon LiDAR sensors merged with a Livox Hub.

To replicate this exact scenario, it is recommended to use the official RealSense ROS wrapper as well as the Livox ROS wrapper, they provide the data already published through topics, and deal with little details we don't have to worry about.

The depth camera deprojection is CUDA accelerated.

## Dependencies
- PCL
- Eigen 3
- CUDA

## Build
- Install the dependencies.
- Clone the package to a ROS workspace.
- Build the workspace: ```catkin_make```.
- Source the workspace setup: ```source devel/setup.bash```.
- Start the nodes as needed.

## Topics
### Subscribed topics
- ```/camera/color/camera_info```: color camera intrinsics, etc.
- ```/camera/color/image_raw```: color camera image.
- ```/camera/depth/camera_info```: depth camera intrinsics, etc.
- ```/camera/depth/image_raw```: depth camera image.

### Published topics
- ```/pointcloud```: merged PointCloud.
