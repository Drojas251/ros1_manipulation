#!/usr/bin/env bash

docker run -it \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -e DISPLAY \
  --privileged \
  ros_manip_app \
  bash