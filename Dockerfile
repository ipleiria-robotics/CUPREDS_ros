FROM carlostojal/cuda-ros:noetic-cuda12.1.1-ubuntu20.04

# environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
ENV PATH=/usr/local/cuda/bin:$PATH

# install dependencies
RUN apt update
RUN apt install -y \
    build-essential \
    ros-noetic-sensor-msgs \
    ros-noetic-tf2-sensor-msgs \
    python3-catkin-tools \
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev \
    vim \
    git \
    xauth \
    wget \
    doxygen

# create the workspace
RUN mkdir -p /home/labrob/catkin_ws/src/pcl_aggregator

# clone, build and install the core library
WORKDIR /home/labrob
RUN git clone -b dev https://github.com/carlostojal/pcl_aggregator_core.git
WORKDIR /home/labrob/pcl_aggregator_core
# build the library
RUN mkdir build
WORKDIR /home/labrob/pcl_aggregator_core/build
RUN cmake ..
RUN make -j8
# install
RUN make install

WORKDIR /home/labrob/catkin_ws