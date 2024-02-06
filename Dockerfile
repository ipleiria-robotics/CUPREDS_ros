FROM carlostojal/cuda-ros:noetic-cuda12.1.1-ubuntu20.04

ARG LIBRARY_SOURCE_PATH=../CUPREDS_core

# environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
ENV PATH=/usr/local/cuda/bin:$PATH

# install dependencies
RUN apt update
RUN apt upgrade -y
RUN apt install -y \
    build-essential \
    python3 \
    python3-catkin-tools \
    python3-rosdep \
    libpcl-dev \
    libopencv-dev \
    libeigen3-dev \
    vim \
    git \
    xauth \
    wget \
    doxygen \
    gdb
RUN apt install -y \
    ros-noetic-cv-bridge \
    ros-noetic-sensor-msgs \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-pcl-conversions \
    ros-noetic-geometry-msgs \
    ros-noetic-foxglove-bridge

# copy the library from host to container
WORKDIR /home/labrob/CUPREDS_core
COPY ${LIBRARY_SOURCE_PATH} /home/labrob/CUPREDS_core
# build the library
RUN mkdir build
WORKDIR /home/labrob/CUPREDS_core/build
RUN cmake ..
RUN make -j8
# install
RUN make install

# copy the ros package and build it
RUN mkdir -p /catkin_ws/src/CUPREDS_ros
COPY ./CUPREDS_ros /catkin_ws/src/CUPREDS_ros
WORKDIR /catkin_ws
# build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make"

# expose ROS master port
EXPOSE 11311

# expose foxglove bridge port
EXPOSE 8765

# launch the aggregator
CMD /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    source /catkin_ws/devel/setup.bash && \
    roslaunch cupreds docker_demo.launch"