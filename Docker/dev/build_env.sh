#!/usr/bin/env bash

# docker dir
docker_directory=$(pwd)

# Move back to workspace dir
cd ../../..

ros_1_manip_package="ros1_manipulation"

# Check that ros1_manipulation and universal_robotics is in this dir
if [ -d "$ros_1_manip_package" ]; then
    ur_package="universal_robot"

    if [ -d "$ur_package" ]; then
        echo "$ur_package  repo already exists...skipping clone"
    else
        echo "Cloning UR Packages"
        git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
    fi
    
else
    current_directory=$(pwd)
    echo "Directory '$ros_1_manip_package' does not exist in '$current_directory' "
fi

cd $docker_directory

# This allows the Docker container to connect to the X server
xhost +local:docker

# Build Docker File
docker build -t ros_manip_dev .