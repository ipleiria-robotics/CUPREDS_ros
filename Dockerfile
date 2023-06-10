FROM carlostojal/cuda-ros:noetic-cuda12.1.1-ubuntu20.04

# environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
ENV PATH=/usr/local/cuda/bin:$PATH

# install dependencies
RUN apt update
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
    doxygen

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

# init rosdep
RUN rosdep init
RUN rosdep update

# copy the ros package and build it
RUN mkdir -p /catkin_ws/src/pcl_aggregator
COPY . /catkin_ws/src/pcl_aggregator
WORKDIR /catkin_ws
# install dependencies
RUN rosdep install --from-paths /catkin_ws --ignore-src --rosdistro noetic -y
# build
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make"

# launch the aggregator
CMD /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    source /catkin_ws/devel/setup.bash && \
    roslaunch pcl_aggregator aggregator.launch"