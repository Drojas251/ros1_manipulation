# ROS1 Manipulation Examples

## Development Env
- Ubuntu 20.04
- ROS Noetic
- Docker

## Docker Install
This project uses docker to install and build the environment. There are two
ways to build this project:
1.) App mode
2.) Dev mode

### App mode
In app mode, one will be able to quickly build the environment and run the provided examples. 
To install and build the app, run the following commands:

```
cd Docker/app
./build_env.sh
./launch_env.sh 
```

This will build and run the docker container. Navigate to /home/ros1_ws in the docker container, and run:

```
source devel/setup.bash
```

Now you are ready to run the examples provided in the Examples section. 

### Dev mode
In dev mode, one will be able to build the environment and mount the workspace directory in the local machine to the workspace directory in the docker container. This will allow one to make changes to the code on your local machine, and execute those changes in the docker container. This mode is recommended to those who want to extend or change the code in any way. To install in dev mode, run the following commands:

```
cd Docker/dev
./build_env.sh
./launch_env.sh
```

This will build and run the docker container. Navigate to /home/ros1_ws in the docker container, and run:

```
sudo apt-get update -y
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```
Now you are ready to run the examples provided in the Examples section. 