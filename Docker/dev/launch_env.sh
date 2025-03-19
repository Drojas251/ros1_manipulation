#!/usr/bin/env bash

# This allows the Docker container to connect to the X server
xhost +local:docker

# docker dir
docker_directory=$(pwd)

# Move back to workspace dir
cd ../../..
ws_directory=$(pwd)

# Check that packages exists
ros_1_manip_package="ros1_manipulation"
ur_package="universal_robot"

if [ -d "$ros_1_manip_package" ]; then
    if [ -d "$ur_package" ]; then
      docker run -it \
        --net=host \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v $ws_directory:/home/ros1_ws/src \
        -v /dev:/dev \
        -e DISPLAY \
        --privileged \
        ros_manip_dev \
        bash
    else
        echo "$ur_package does not exit. Please clone into dir: $ws_directory"
    fi
else
  echo "$ros_1_manip_package does not exit. Please clone into dir: $ws_directory"   
fi