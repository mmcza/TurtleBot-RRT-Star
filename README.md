# TurtleBot4 with RRT* path planning algorithm

## Table of Contents

- [TurtleBot4 with RRT\* path planning algorithm](#turtlebot4-with-rrt-path-planning-algorithm)
  - [Table of Contents](#table-of-contents)
  - [1. Installation](#1-installation)
    - [In Ubuntu 22.04 with ROS2 Humble](#in-ubuntu-2204-with-ros2-humble)
      - [1. Create the workspace and clone the repository](#1-create-the-workspace-and-clone-the-repository)
      - [2. Install the TurtleBot4 Simulator (as in TurtleBot's manual)](#2-install-the-turtlebot4-simulator-as-in-turtlebots-manual)
    - [Using Docker with NVIDIA Container Toolkit](#using-docker-with-nvidia-container-toolkit)
      - [1. Create the workspace and clone the repository](#1-create-the-workspace-and-clone-the-repository-1)
      - [2. Build the container](#2-build-the-container)
      - [3. Start the container](#3-start-the-container)


## 1. Installation

### In Ubuntu 22.04 with ROS2 Humble

#### 1. Create the workspace and clone the repository

```
mkdir -p ~/turtlebot_ws/src/ && cd ~/turtlebot_ws/src/
```
```
git clone https://github.com/mmcza/TurtleBot-RRT-Star
```

#### 2. Install the TurtleBot4 Simulator ([as in TurtleBot's manual](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html))

Dev Tools
``` 
sudo apt install ros-dev-tools && sudo apt-get update && sudo apt-get install wget
```
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
```
Ignition Gazebo
```
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
```
```
sudo apt-get update && sudo apt-get install ignition-fortress
```
Debian package
```
sudo apt update && sudo apt install ros-humble-turtlebot4-simulator
```

### Using Docker with NVIDIA Container Toolkit

#### 1. Create the workspace and clone the repository

```
mkdir -p ~/turtlebot_ws/src/ && cd ~/turtlebot_ws/src/
```
```
git clone https://github.com/mmcza/TurtleBot-RRT-Star
```

#### 2. Build the container 

```
cd TurtleBot-RRT-Star && docker build -t turtlebot4_rrt_star .
```

#### 3. Start the container

```
bash start_container.sh 
```

> [!NOTE]
> The `turtlebot_ws` directory is shared between the host and container. In the result files inside of it might require sudo privileges to save any changes.

> [!NOTE]
> Dockerfile and script for running container are based on [Rafał Staszak's repository](https://github.com/RafalStaszak/NIMPRA_Docker/)
