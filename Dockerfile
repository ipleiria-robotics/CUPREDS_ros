FROM osrf/ros:noetic-desktop-full

# install dependencies
RUN apt update && apt install libpcl-dev libopencv-dev libeigen3-dev wget neofetch vim geany -y

# install cuda
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb
RUN dpkg -i cuda-keyring_1.0-1_all.deb
RUN apt update && apt install cuda -y
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# copy the package
RUN mkdir -p ~/catkin_ws/src
COPY . ~/catkin_ws/src/pcl_aggregator

WORKDIR ~/catkin_ws

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute