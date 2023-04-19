# Navigation Utilities
This package offers a series of ROS services that help the robot navigate robust and smoothly through different environments.

**Table of Contents**

- [Navigation Utilities](#navigation-utilities)
- [Installation](#installation)
  * [Requirements](#requirements)
  * [Dependencies](#dependencies)
    + [Libraries](#libraries)
    + [ROS Packages](#ros-packages)
    + [ROS Actions](#ros-actions)
  * [Install](#install)
- [Execution](#execution)
- [Usage](#usage)
  * [Navigation Graph](#navigation-graph)
    + [Places File](#places-file)
    + [Edges File](#edges-file)
  * [Services](#services)
    + [set_current_place_srv](#set_current_place_srv)
    + [go_to_relative_point_srv](#go_to_relative_point_srv)
    + [go_to_place_srv](#go_to_place_srv)
    + [start_random_navigation_srv](#start_random_navigation_srv)
    + [add_place_srv](#add_place_srv)
    + [follow_you_srv](#follow_you_srv)
    + [robot_stop_srv](#robot_stop_srv)
    + [spin_srv](#spin_srv)
  * [Topics](#topics)
    + [simple_feedback](#simple_feedback)
    + [complete_feedback](#complete_feedback)
- [Troubleshooting](#troubleshooting)
  * [ModuleNotFoundError](#modulenotfounderror)
    + [Numpy](#numpy)
    + [NetworkX](#networkx)
	+ [rospkg](#rospkg)
	+ [navigation_msgs](#navigation_msgs)
  * [CMake Error move_base_msgs](#CMake-Error-move_base_msgs)
  * [Dynamic module does not define module export function](#dynamic-module-does-not-define-module-export-function)


# Installation
## Requirements

- Linux Ubuntu 18.04
- ROS Melodic
- Python >= 3.6

## Dependencies
### Libraries

- [Numpy][Numpy]
- [NetworkX][Networkx]

### ROS Packages
- [rospkg][rospkg]
- [navigation_msgs][navigation_msgs]

### ROS Actions

- [move_base][move_base]

## Install

1.  Clone the repository (recommended location is ~).

```bash
  cd ~
  ```

  ```bash
  git clone https://github.com/SinfonIAUniandes/navigation_utilities.git
  ```

2.  Move to the root of the workspace.

  ```bash
  cd ~/navigation_utilities
  ```

3.  Build the workspace.

  ```bash
  catkin_make
  ```
  ```bash
  source devel/setup.bash
  ```

# Execution

When roscore is available run:

 ```bash
  rosrun navigation_utilities NavigationUtilities.py
  ```

# Usage
## Navigation Graph
The navigation graph is an undirected graph where each of the vertices represents a place on the map and each edge represents that two places are connected. 
For open spaces and long distances between goals the use of a navigation graph is recommended. This graph is automatically created based on places.txt and edges.txt available in */resources*.
### Places File

Each lines represents a place (possible goal) and must have the next format: *(name, x, y, theta)*. For example:

`kitchen, 0.5, 0.6, 90`

**Clarification:** *x* and *y* are cartesian coordinates and *theta* is a yaw angle in grades respect to the map frame.

### Edges File

Each line represents a connection between two places and must have the next format: *(name1, name2)*. For example, to connect kitchen and bathroom:

`kitchen, bathroom`

## Services

Navigation_utilities offers the following services:

### set_current_place_srv

+ **Description:**
This service allows to set the pose of the robot in one of the known places (e.g., kitchen, bathroom, bedroom).

+ **Service file:**
*set_current_place_srv.srv*
    + **Request**: 
		+ name (string): Name of the place to set.
	+ **Response**:
		+ answer (string): Indicates if the service was approved or not.
		
+ **Call service example:**

 ```bash
 rosservice call /navigation_utilities/set_current_place_srv "name: 'kitchen'"
  ```

------------

### go_to_relative_point_srv

+ **Description:**
This service allows the robot to navigate to a target that is in a position relative from it.
For example, move the robot 2 meter towards the *x* coordinate axis, 1 meter towards the *y* coordinate axis and rotate 30 degrees counterclockwise.
+ **Service file:**
*go_to_relative_point_srv.srv*
    + **Request**: 
		+ x (float64): x coordinate relative to the robot position.
		+ y (float64):  y coordinate relative to the robot position.
		+ theta (float32): yaw angle to rotate (in degrees).
	+ **Response**:
		+ answer (string): Indicates if the service was approved or not.
		
+ **Call service example:**

```bash
rosservice call /navigation_utilities/go_to_relative_point_srv "x: 2.0
y: 1.0
theta: 30.0"
```

------------

### go_to_place_srv

+ **Description:**
This service allows the robot to navigate to a specific place (e.g., kitchen, bathroom, bedroom). The robot will try it 3 times before aborting. 

+ **Service file:**
*go_to_place_srv.srv*
    + **Request**: 
		+ name (string): Name of the place to navigate.
		+ graph (int8): Indicates if the navigation_graph is used or not.
	+ **Response**:
		+ answer (string): Indicates if the service was approved or not.

+ **Call service example:**

 ```bash
 rosservice call /navigation_utilities/go_to_place_srv "name: 'kitchen'
graph: 1"
  ```
------------

### start_random_navigation_srv

+ **Description:**
This service allows the robot to randomly navigate through a given environment (static map). To stop the random navigation call the [robot_stop_srv](#robot_stop_srv) service.

+ **Service file:**
*start_random_navigation_srv.srv*
    + **Request**: 
		+ void
	+ **Response**:
		+ answer (string): Indicates if the service was approved or not.

+ **Call service example:**

 ```bash
rosservice call /navigation_utilities/start_random_navigation_srv "{}"
  ```

 ------------


### add_place_srv

+ **Description:**
Given the name and the coordinates of a place it is saved as a node in the Navigation Graph. If 'persist' perameter is 1, the place will be saved for future occasions. If it is 0, it will remember the location until navigation_utilities is disabled.


+ **Service file:**
*add_place_srv.srv*
    + **Request**: 
		+ name (string): Name of the place to navigate.
		+ persist (int8): Indicates if the place is persisted or not.
		+ edges (string[]): Indicates the places that are connected to the location.
	+ **Response**:
		+ answer (string): Indicates if the service was approved or not.

+ **Call service example:**

 ```bash
rosservice call /navigation_utilities/add_place_srv "name: 'kitchen'
persist: 1
edges: [beggining, dinning room]"
  ``` 
  
------------

### follow_you_srv

+ **Description:**
Moves to an objective while a person is touching its head.


+ **Service file:**
*follow_you_srv.srv*
    + **Request**: 
		+ place (string): Name of the place to navigate.
	+ **Response**:
		+ answer (string): Indicates if the service was approved or not.

+ **Call service example:**

 ```bash
rosservice call /navigation_utilities/follow_you_srv "place: 'kitchen'
graph: 1"
  ``` 
  
------------


### robot_stop_srv

+ **Description:**
This service allows to stop the robot immediately after being requested. It cancels all the goals sent it to the move_base action client. If random navigation is active this disables it and if the robot is spinning it will be stopped.

+ **Service file:**
*robot_stop_srv.srv*
    + **Request**: 
		+ void
	+ **Response**:
		+ result (string): Indicates if the service was approved or not.

+ **Call service example:**

 ```bash
rosservice call /navigation_utilities/robot_stop_srv "{}"
  ```
  ------------


### spin_srv

+ **Description:**
Makes the robot turn depending on the angle (in degrees) that has been entered, the path will be the shortest to reach the objective.


+ **Service file:**
*spin_srv.srv*
    + **Request**: 
		+ degrees (float64): Angle that the robot will rotate (angle in degrees that must be in the range [-360,360]).

	+ **Response**:
		+ answer (string): Indicates if the service was approved or not.

+ **Call service example:**

 ```bash
rosservice call /navigation_utilities/spin_srv "degrees: 90.0"

  ``` 
  
------------
## Topics
### simple_feedback
- **Description:**
In this topic the navigation status is published. 

| Status      | Meaning |
| :---------: | :-----:|
| 0  | No goal |
| 1     |  Navigating |
| 2      |  Final goal reached |
| 3      |   Goal aborted |
| 4      |   Cancel Request |

- **Message File:**
*simple_feedback_msg.msg*
    + **Content**: 
		+ navigation_status (int32): Indicates the navigation status.

------------

### complete_feedback
- **Description:**
In this topic all the information related with the navigation is published. 


- **Message File:**
*move_base_msgs/MoveBaseFeedback.msg*
    + **Content**: 
		+ base_position (geometry_msgs/PoseStamped): Indicates all the information related with the navigation.

# Troubleshooting
Below is the solution to a series of problems that can occur when trying to use the navigation_utilities.
## ModuleNotFoundError
This error happens when one of the libraries or packages described in the dependencies section is not installed. 

For the python libraries it is recommended to install them through pip3.

*If you do not have this tool, you can install it through the following command:*

```bash
sudo apt install python3-pip
```
### Numpy

Installing Numpy through pip:

```bash
pip3 install numpy
```

### NetworkX
Installing NetworkX through pip:

```bash
pip3 install networkx
```

### rospkg
Installing some prerequisites to use Python3 with ROS.
```bash
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```

### navigation_msgs
For installing navigation_msgs follow the instructions found in the repository: [navigation_msgs][navigation_msgs]

## CMake Error move_base_msgs
This error happens because it is not installed move_base msg, therefore need to install it. It can be install it through the following command:

```bash
sudo apt-get install ros-[DISTRO]-navigation
```

## Dynamic module does not define module export function

This error happens because tf2_ros was compiled for python2 instead of python3. For solving this:

1. Install some prerequisites to use Python3 with ROS.

```bash
sudo apt update
```
```bash
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```
2. Move to the root of your workspace.

3. Prepare your workspace.

```bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
```
4. Compile for Python 3.

```bash
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
```

5. Source your workspace.

```bash
source devel/setup.bash
```



[Numpy]: https://numpy.org "Numpy"
[Networkx]: https://networkx.org "Networkx"
[navigation_msgs]: https://github.com/SinfonIAUniandes/navigation_msgs "navigation_msgs"
[move_base]: http://wiki.ros.org/move_base "move_base"

[rospkg]: http://wiki.ros.org/rospkg "rospkg"
