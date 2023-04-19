# Navigation Messages
This package is designed to allocate navigation messages used by navigation utilites for SinfonIA.



**Table of Contents**
- [Navigation Messages](#navigation-messages)
- [Installation](#installation)
  * [Requirements](#requirements)
  * [Dependencies](#dependencies)
    + [ROS Packages](#ros-packages)
  * [Install](#install)

# Installation
## Requirements

- Linux Ubuntu
- ROS

## Dependencies

### ROS Packages

- [geometry_msgs][geometry_msgs]

## Install

1.  Clone the repository in the same workspace where is located the navigation_utilities.

  ```bash
  git clone https://github.com/SinfonIAUniandes/navigation_msgs
  ```

2.  Move to the root of the workspace.

  ```bash
  cd ~/.../navigation_msgs
  ```

3.  Build the workspace.

  ```bash
  catkin_make
  ```
  ```bash
  source devel/setup.bash
  ```

[geometry_msgs]: http://wiki.ros.org/geometry_msgs "geometry_msgs"