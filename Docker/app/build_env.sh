#!/usr/bin/env bash

# This allows the Docker container to connect to the X server
xhost +local:docker

# Build Docker File
docker build -t ros_manip_app .