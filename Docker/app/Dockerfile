
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

# Clone repos
WORKDIR /home/ros1_ws/src
RUN git clone https://github.com/Drojas251/ros1_manipulation.git \
  && git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git

# Install ros deps and build workspace
WORKDIR /home/ros1_ws
RUN apt-get update -y
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
RUN /bin/bash -c "source devel/setup.bash"