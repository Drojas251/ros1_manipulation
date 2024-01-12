
FROM ros:noetic

# Install required packages
RUN apt-get update && apt-get install -y \
    ros-noetic-moveit  \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    python3-rosdep \
    ros-noetic-rviz \
    ros-noetic-behaviortree-cpp-v3 \
    python3-rosdep \
    git-all \
    xauth \
    && apt-get clean

# Install python packages
RUN apt-get update -y
RUN apt install python3-pip -y
RUN pip install pandas
RUN pip install tensorflow
RUN pip install scikit-learn
RUN pip install colorama

# Source ROS env
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

WORKDIR /home/ros1_ws/