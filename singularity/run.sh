#!/usr/bin/env bash

singularity shell --nv \
    --env DISPLAY=$DISPLAY \
    --bind .:/home/labrob/catkin_ws/src/pcl_aggregator \
    --bind /tmp/.X11-unix:/tmp/.X11-unix \
    --bind $(eval echo "~$SUDO_USER")/.Xauthority:/home/labrob/.Xauthority \
    --overlay singularity/overlay_dir/:/home/labrob/catkin_ws \
    singularity/pcl_aggregator.sif
