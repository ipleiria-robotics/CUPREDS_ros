docker run pcl_aggregator \
    --gpus all \
    --mount type=bind,source=.,target=~/catkin_ws/src/pcl_aggregator \
    -it /bin/bash