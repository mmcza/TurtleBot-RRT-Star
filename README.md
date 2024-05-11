# TurtleBot with RRT* path planning algorithm

## Table of Contents

- [TurtleBot with RRT\* path planning algorithm](#turtlebot-with-rrt-path-planning-algorithm)
  - [Table of Contents](#table-of-contents)
  - [1. Installation](#1-installation)
    - [In Ubuntu 22.04 with ROS2 Humble](#in-ubuntu-2204-with-ros2-humble)
      - [1. Create the workspace and clone the repository](#1-create-the-workspace-and-clone-the-repository)
      - [2. Install the Navigation2 package and TurtleBot3 Simualtion](#2-install-the-navigation2-package-and-turtlebot3-simualtion)
    - [Using Docker with NVIDIA Container Toolkit](#using-docker-with-nvidia-container-toolkit)
      - [1. Create the workspace and clone the repository](#1-create-the-workspace-and-clone-the-repository-1)
      - [2. Build the container](#2-build-the-container)
      - [3. Start the container](#3-start-the-container)
  - [2. Running the simulation](#2-running-the-simulation)
    - [Override the params file](#override-the-params-file)
    - [Workspace preparation](#workspace-preparation)
    - [Building the package](#building-the-package)
    - [Launching the simulation](#launching-the-simulation)
  - [3. About the RRT\* Algorithm](#3-about-the-rrt-algorithm)
  - [4. RRT\* Algorithm Implementation](#4-rrt-algorithm-implementation)
  - [5. Encountered issues and solutions](#5-encountered-issues-and-solutions)
    - [Gazebo not starting](#gazebo-not-starting)
    - [Unable to use custom plugin for Nav2 path planning](#unable-to-use-custom-plugin-for-nav2-path-planning)
    - [Memory errors](#memory-errors)
  - [6. Reference](#6-reference)



## 1. Installation

### In Ubuntu 22.04 with ROS2 Humble

#### 1. Create the workspace and clone the repository

```
mkdir -p ~/turtlebot_ws/src/ && cd ~/turtlebot_ws/src/
```
```
git clone https://github.com/mmcza/TurtleBot-RRT-Star
```

#### 2. Install the Navigation2 package and TurtleBot3 Simualtion 

```
sudo apt update
```

```
sudo apt install ros-humble-navigation2
```
```
sudo apt install ros-humble-nav2-bringup
```

```
sudo apt install ros-humble-turtlebot3-gazebo
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
cd TurtleBot-RRT-Star && docker build -t rrt_star_tb3 .
```

#### 3. Start the container

```
bash start_container.sh 
```

> [!NOTE]
> The `turtlebot_ws` directory is shared between the host and container. In the result files inside of it might require sudo privileges to save any changes.

> [!NOTE]
> Dockerfile and script for running container are based on [Rafał Staszak's repository](https://github.com/RafalStaszak/NIMPRA_Docker/)

## 2. Running the simulation

> [!NOTE]
> Currently in this repositorium there is implemented the RRT Algorithm.

### Override the params file
```
cp /path/to/the/repository/TurtleBot-RRT-Star/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```
Example: in Docker it is:
```
cp ~/Shared/src/TurtleBot-RRT-Star/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

### Workspace preparation

```
source /opt/ros/humble/setup.bash
```
```
export TURTLEBOT3_MODEL=waffle
```
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```
```
source /usr/share/gazebo/setup.bash
```

### Building the package
When inside the workspace:
```
colcon build --symlink-install
```

```
source /opt/ros/humble/setup.bash
```

```
source install/setup.bash
```

### Launching the simulation
To start the simulation you need to execute the following command
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml 
```
It may take some time to start the Gazebo (if it's first time running than even more) and once it's working you have to check in the Gazebo where on the map the robot is located and in RVIZ click `2D Pose Estimate` button and later place it in the place of the robot (adjust the arrow to show the orientation of the robot). After that click `Nav2 Goal` and once again click on the map (you can adjust the end orientation with the arrow). After that the robot should start moving (and in RVIZ you should see the planned path).

## 3. About the RRT* Algorithm

The difference between RRT* and default RRT Algorithm is the fact that RRT* can rewire the nodes when new one appears and cost of going through it is lower than through an existing one. 

Pseudocode for RRT* is shown below [[1]](#1).

![RRT_Star_Algorithm](/Pictures/RRT_Star_pseudocode.png)

## 4. RRT* Algorithm Implementation

## 5. Encountered issues and solutions

### Gazebo not starting

The issue might be caused by a corrupted package or by not sourcing the gazebo.

Solution might be using the following command
```
source /usr/share/gazebo/setup.bash
```
or using a Docker container with clean system.

### Unable to use custom plugin for Nav2 path planning

This issue (caused by wrong timestamps) made that it was impossible to set initial pose of the robot (and as a result to plan a path).

Example of the error message:
```
[component_container_isolated-6] [WARN] [1715109253.077329208] [amcl]: Failed to transform initial pose in time (Lookup would require extrapolation into the future.  Requested time 1715109253.077081 but the latest data is at time 70.142000, when looking up transform from frame [odom] to frame [base_footprint])
```

Solution was (as mentioned in [this comment](https://github.com/open-navigation/navigation2_tutorials/issues/25#issuecomment-1179464652)) to comment in the cpp file the lines that captured time.

### Memory errors

The issue was that pointers were pointing address in a vector that could be relocated.

The solution was to reserve the memory.

## 6. Reference

<a id="1">[1]</a>
Karaman, Sertac and Frazzoli, Emilio. Sampling-based Algorithms for Optimal Motion Planning. 2011, [https://doi.org/10.48550/arXiv.1105.1186](https://doi.org/10.48550/arXiv.1105.1186)
