# SafeForest Point Cloud Aggregator

This package is intended to provide functionality to aggregate pointclouds provenient from different sources. For instance, a deprojected depth camera and LiDAR sensors. Currently, the depth camera deprojection is done solely with CPU processing, but a version with CUDA computing is under development.

The RGBD deprojector subscribes for both a color image and a depth aligned to color image. Note that the depth frame must really be aligned, because the color frame is used to add color to the points.

The hardware tested during the development of this package was the RealSense D435i depth camera and 3 Livox Horizon LiDAR sensors merged with a Livox Hub.

To replicate this exact scenario, it is recommended to use the official RealSense ROS wrapper as well as the Livox ROS wrapper. They provide the data already published through topics, and deal with little details we don't have to worry about.

## Dependencies
- PCL
- Eigen 3
- CUDA Toolkit (+ CUDA-enabled GPU)

## Build
- Install the dependencies.
- Clone the package to a ROS workspace.
- Build the workspace: ```catkin_make```.
- Source the workspace setup: ```source devel/setup.bash```.
- Start the nodes as needed.

## Nodes
### pcl_aggregator_node
#### Parameters
- ```n_pointclouds```: Number of PointClouds to be subscribed. Default: 2.
- ```publish_rate```: Rate at which to publish the merged pointcloud specified in Hz. Best effort, this is the maximum. Default: 20.
#### Subscribed topics
- ```pointcloudn```: PointCloud topics, where $0 \leq n < n_{pointclouds}$. Example: ```pointcloud0```. Type: sensor_msgs/PointCloud2.
#### Published topics
- ```merged_pointcloud```: Merged PointCloud data. Publish at the rate specified on the ```publish_rate``` parameter. Type: sensor_msgs/PointCloud2.

### rgbd_deprojector_node
#### Subscribed topics
- ```/camera/aligned_depth_to_color/camera_info```: Depth camera information data. Type: sensor_msgs/CameraInfo.
- ```/camera/aligned_depth_to_color/image_raw```: Depth frame aligned to the RGB frame. Type: sensor_msgs/Image.
- ```/camera/color/image_raw```: Color frame. Type: sensor_msgs/Image.
#### Published topics
- ```/pointcloud```: deprojected PointCloud data. Type: sensor_msgs/PointCloud2.
