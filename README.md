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
  - [2. Running the simulation](#2-running-the-simulation)
    - [Change the used path planning plugin](#change-the-used-path-planning-plugin)
    - [Source the ROS files](#source-the-ros-files)
    - [Launch the simulation](#launch-the-simulation)
    - [Selecting goal and running the robot](#selecting-goal-and-running-the-robot)
    - [TODO: add a gif](#todo-add-a-gif)
  - [5. Encountered issues and solutions](#5-encountered-issues-and-solutions)
    - [Gazebo running slowly](#gazebo-running-slowly)


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

## 2. Running the simulation

> [!NOTE]
> Currently in this repositorium there is a tutorial package from [navigation tutorial](https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html)

### Change the used path planning plugin


In general 
```
cp /path/to/turtlebot_ws/src/TurtleBot-RRT-Star/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```
If you're using the Docker container

```
cp ~/Shared/src/TurtleBot-RRT-Star/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

### Source the ROS files

```
source /opt/ros/humble/setup.bash
```

### Launch the simulation
```
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true
```
> [!NOTE]
> Here is an issue - when I add flag `params_file:=~/Shared/src/TurtleBot-RRT-Star/nav2_params.yaml` it says `No such file or directory` and currently I have no idea how to solve it - that's why I override the file inside `/opt/ros/humble/share/nav2_bringup/params/`. 

### Selecting goal and running the robot

Once the gazebo and rviz are running, you have to press the `play` button in bottom left corner in gazebo.

After that in rviz you have to select the starting position of robot using `2D Pose Estimate` button on top of the window and clicking in proper location and use the arrow to display robot's orientation. To select robot's destination click `Nav2 Goal` button and select the desired point on the map.

### TODO: add a gif

Now you can see that the robot is moving towards the goal and you can see the planned path in rviz.

## 5. Encountered issues and solutions

### Gazebo running slowly

The issue was that the simulation in Gazebo was running really slowly - around 10-15% RTF (the simulation was running approx. 7-10 times slower than real time).

The solution is to change the PRIME Profiles in NVIDIA Settings to Performance Mode (as suggested [in this reply](https://github.com/turtlebot/turtlebot4_simulator/issues/38#issuecomment-1548451019)). To solve this problem you need to type the following command in host system:

```
nvidia-settings
```

After rebooting the computer, RTF went up to a level of about 80-95%.