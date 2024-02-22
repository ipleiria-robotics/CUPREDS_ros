Student:<br>
Carlos Tojal<sup>1,2</sup><br>

Advisors:<br>
Luís Conde Bento<sup>1,2</sup><br>

<sup>1</sup> Polytechnic of Leiria, Leiria, Portugal;<br>
<sup>2</sup> Institute of Systems and Robotics, Coimbra, Portugal 

# CUPREDS – CUDA-Accelerated, Multi-threaded Point Cloud Registration from Heterogeneous Sensors with Efficient Data Structures

This package is intended to provide functionality to aggregate pointclouds provenient from different sources. For instance, a deprojected depth camera and LiDAR sensors. Currently, the depth camera deprojection is done solely with CPU processing, but a version with CUDA computing is under development.

The PointCloud aggregator basically does the registration of the different sources, and manages the storage of the corresponding PointClouds.

The registration is ICP-based, and the merged PointCloud is downsampled using voxel grid.

The RGBD deprojector subscribes for both a color image and a depth aligned to color image. Note that the depth frame must really be aligned, because the color frame is used to add color to the points.

The hardware tested during the development of this package was the RealSense D435i depth camera and 3 Livox Horizon LiDAR sensors merged with a Livox Hub.

To replicate this exact scenario, it is recommended to use the official RealSense ROS wrapper as well as the Livox ROS wrapper. They provide the data already published through topics, and deal with little details we don't have to worry about.

## Dependencies
- CUPDREDS\_core]
- PCL
- Eigen 3
- CUDA Toolkit (+ CUDA-enabled GPU)
- OpenMP
- OpenCV

## Build
### Bare metal
- Install the dependencies.
- Clone the package to a ROS workspace.
- Build the workspace: ```catkin_make```.
- Source the workspace setup: ```source devel/setup.bash```.
- Start the nodes as needed.

### Singularity
- Install singularity. It is recommended to follow the official documentation.
- Install X.org X server on the host, if not already installed.
- Run the script ```singularity/build.sh```. It can take some time, wait for it to finish.
- Run the script ```singularity/run.sh``` **as superuser**. You should now be inside the container.
- Run ```source /opt/ros/noetic/setup.bash``` and ```catkin_make``` to build the ROS wrapper.
- From now on, it's up to you :)

### Docker
- Install Docker. It is recommended to follow the official documentation.
- Setup the NVIDIA Container Toolkit for Docker ([guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#install-guide)).
- Clone the core library repository CUPREDS_core to the same level as this directory.
- From the directory containing `CUPREDS\_core` and `CUPREDS\_ros`, run `docker build -t cupreds:latest -f CUPREDS\_ros/Dockerfile .`. This may take a while, just wait. It will install all the dependencies and build the core library, install it, and build the package.
- You can start the container with the running nodes with `docker run --rm --runtime=nvidia --gpus all --network=host -it cupreds`.

## Nodes
### pcl\_aggregator\_node
#### Parameters
- ```n_pointclouds```: Number of PointClouds to be subscribed. Default: 1.
- ```max_pointcloud_age```: The PointClouds considered to register are the ones newer than the age specified on this parameter in seconds. Default: 2.
- ```max_memory```: The maximum memory to be used by the PointCloud in MB. Default: 5000.
- ```robot_base```: The frame to considered as robot base. Default: base_link.
- ```publish_rate```: The publish rate in Hz. Default: 10.
#### Subscribed topics
- ```pointcloudn```: PointCloud topics, where $0 \leq n < n_{pointclouds}$. Example: ```pointcloud0```. Type: sensor_msgs/PointCloud2.
#### Published topics
- ```merged_pointcloud```: Merged PointCloud data. Type: sensor_msgs/PointCloud2.
#### Required tf transforms
- ```robot_base```->```pointcloudn```, where $0 \leq n < n_{pointclouds}$ and ```pointcloudn``` refers to the frame id used by that sensor.
#### Services
- ```snapshot```: Save locally a ```.pcd``` file of the current PointCloud generated.
    - Input: filename (string)
    - Output: filename (string)

### rgbd\_deprojector\_node
#### Subscribed topics
- ```/camera/aligned_depth_to_color/camera_info```: Depth camera information data. Type: sensor_msgs/CameraInfo.
- ```/camera/aligned_depth_to_color/image_raw```: Depth frame aligned to the RGB frame. Type: sensor_msgs/Image.
- ```/camera/color/image_raw```: Color frame. Type: sensor_msgs/Image.
#### Published topics
- ```/pointcloud```: deprojected PointCloud data. Type: sensor_msgs/PointCloud2.
