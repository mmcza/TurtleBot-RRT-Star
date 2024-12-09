# TurtleBot with RRT* path planning algorithm

![RRT_Star.gif](/Pictures/RRT_Star.gif)

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
    - [Changing frequency of path planning (optional)](#changing-frequency-of-path-planning-optional)
    - [Building the package](#building-the-package)
    - [Launching the simulation](#launching-the-simulation)
  - [3. About the RRT\* Algorithm](#3-about-the-rrt-algorithm)
  - [4. RRT\* Algorithm Implementation](#4-rrt-algorithm-implementation)
    - [configuration and lifecycle management](#configuration-and-lifecycle-management)
    - [calculate radius](#calculate-radius)
    - [vertices inside circle](#vertices-inside-circle)
    - [nearest neighbor and connectivity check](#nearest-neighbor-and-connectivity-check)
    - [calculate cost](#calculate-cost)
    - [create plan](#create-plan)
    - [main loop](#main-loop)
    - [returning the optimal path](#returning-the-optimal-path)
  - [5. Encountered issues and solutions](#5-encountered-issues-and-solutions)
    - [Gazebo not starting](#gazebo-not-starting)
    - [Unable to use custom plugin for Nav2 path planning](#unable-to-use-custom-plugin-for-nav2-path-planning)
    - [Memory errors](#memory-errors)
    - [Robot stopping even the path was planned](#robot-stopping-even-the-path-was-planned)
  - [6. Possible development directions](#6-possible-development-directions)
  - [Reference](#reference)



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

### Override the params file
```
cp /path/to/the/repository/TurtleBot-RRT-Star/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```
Example: in Docker it is:
```
cp ~/Shared/src/TurtleBot-RRT-Star/nav2_params.yaml /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

### Changing frequency of path planning (optional)

By default the path planner runs once every second and because RRT* is based on random points the path may differ (with number of points equal $\infty$ the path would be the same as it would be optimal) and it may cause the robot to go in completely different way. The solution can be to change the frequency inside `navigate_to_pose_w_replanning_and_recovery.xml`. Below is an example that disable (by setting rate = 0.0) the periodic replanner but the path may be replanned if robot get stuck.

```
sed -i '11s|.*|        <RateController hz="0.0">|' /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
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

The difference between RRT* and default RRT Algorithm is the fact that RRT* takes cost of the robot moving from one node to another and that it can rewire the nodes when new one appears and cost of going through it is lower than through an existing one. Also basic RRT algorithm stops when there is a path between a node and the end point while RRT* continue for certain number of iterations. As a result RRT* algorithm is asymptotically optimal (with number of nodes going towards $\infty$ it will find the optimal path).

Pseudocode for RRT* is shown below [[1]](#1).

![RRT_Star_Algorithm](/Pictures/RRT_Star_pseudocode.png)

Example of RRT Star in action can be seen below (code available [here](https://github.com/mmcza/Motion-Planning-Methods-and-Algorithms_laboratory/tree/main/rrt_star)) - however the change of radius wasn't implemented and there was added a constant step distance between newly added vertex.
![RRT_Star_example](/Pictures/rrt_star_example.gif)

## 4. RRT* Algorithm Implementation
### [configuration and lifecycle management](https://github.com/mmcza/TurtleBot-RRT-Star/blob/d820402ae66da42e88c468be72f1da76d5bad2d7/src/rrtstar_planner.cpp#L14-L51)

- initalizes the RRTStar planner by setting up necessary parameters, such as the interpolation resolution and costmap.

### [calculate radius](https://github.com/mmcza/TurtleBot-RRT-Star/blob/d820402ae66da42e88c468be72f1da76d5bad2d7/src/rrtstar_planner.cpp#L53-L77)

- calculateBallEadiusConstant computes a constant used for determining the ball radius during the rewire step, based on the free space in the costmap.
- calculateBallRadius calculates the actual ball radius used to find nearby vertices during the rewire step. The radius is determined based on the size of the tree and ensures that it does not exceed a specified maximum connection distance.

 ### [vertices inside circle](https://github.com/mmcza/TurtleBot-RRT-Star/blob/d820402ae66da42e88c468be72f1da76d5bad2d7/src/rrtstar_planner.cpp#L79-L95)
  
  - vertices_inside_circle the function returns a vector of indices representing the vertices that lie within the specified radius from the center point. This is done by iteratively following the parent pointers from the given vertex back to the start vertex.
  
### [nearest neighbor and connectivity check](https://github.com/mmcza/TurtleBot-RRT-Star/blob/d820402ae66da42e88c468be72f1da76d5bad2d7/src/rrtstar_planner.cpp#L96-L131)

- calculate_distance this function calculates the Euclidean distance between a given point and a vertex in the tree.
- nearest_neighbor this function finds the nearest vertex in the tree to a given point.
- connectible checks whether a straight-line path between two vertices is free of obstacles. It determines the number of interpolation steps based on the distance and resolution, and then incrementally checks each point for obstacles in the costmap.

### [calculate cost](https://github.com/mmcza/TurtleBot-RRT-Star/blob/d820402ae66da42e88c468be72f1da76d5bad2d7/src/rrtstar_planner.cpp#L133-L144)
- calculate_cost_from_start computes the total cost from the start vertex to a given vertex by summing the costs along the path.

### [create plan](https://github.com/mmcza/TurtleBot-RRT-Star/blob/d820402ae66da42e88c468be72f1da76d5bad2d7/src/rrtstar_planner.cpp#L146-L195)
- createPlan function generates a path from the start pose to the goal pose using the RRT* algorithm.

### [main loop](https://github.com/mmcza/TurtleBot-RRT-Star/blob/d820402ae66da42e88c468be72f1da76d5bad2d7/src/rrtstar_planner.cpp#L199-L254)
- in the main loop of createPlan function, the algorithm generates a random point within the costmap and creates a new vertex. It finds the nearest existing vertex in the tree, check if it's possible to connect them with a straight line and assigns it as the parent of the new vertex and calculates the cost to reach it. If the connection is possible, it calculates a ball radius to find nearby vertices, adds the new vertex to the tree, and calculates its total cost from the start. The algorithm then attempts to rewire the tree by finding more efficient paths through the new vertex to existing vertices or through existing vertices to the vertex. If the tree is rewired it's updating parent pointers and costs accordingly. If the path is obstructed, the new vertex is discarded, and the iterations is retired.

### [returning the optimal path](https://github.com/mmcza/TurtleBot-RRT-Star/blob/d820402ae66da42e88c468be72f1da76d5bad2d7/src/rrtstar_planner.cpp#L258-L329)

If there is a total of 1000 vertices in the tree, the algorithm terminates the loop and proceeds to find the optimal path to the goal using the vertices that are in the tree by recursively going through parents of the vertices all the way to the start point. This ensures that the algorithm does not run indefinitely and provides a solution within a reasonable amount of time.

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

### Robot stopping even the path was planned

The problem was caused by the fact that next pose in the path had to be in certain radius of the robot so it can go towards it.

The solution was to add more waypoints between each two poses.

## 6. Possible development directions

One of the possibilities is to experiment with the radius of the circle, because when the value is too low and there are many obstacles than the algorithm may return a non optimal path.

Other potential upgrade would be to use RRT* Informed algorithm and limit the space where new vertices are being added after the initial path is found.

The structure of the tree could also be improved to make searching for vertices inside the circle faster. A good example would be to use a k-d tree.

## Reference

<a id="1">[1]</a>
Karaman, Sertac and Frazzoli, Emilio. Sampling-based Algorithms for Optimal Motion Planning. 2011, [https://doi.org/10.48550/arXiv.1105.1186](https://doi.org/10.48550/arXiv.1105.1186)
