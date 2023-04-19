cmake_minimum_required(VERSION 3.5)
project(remote_controller)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  cv_bridge
  std_msgs  
  robot_toolkit_msgs
  sensor_msgs
  Qt5Widgets
  Qt5Gui
  Qt5Core
)


find_package(OpenCV)

catkin_package(
   CATKIN_DEPENDS roscpp rospy roslib std_msgs geometry_msgs robot_toolkit_msgs sensor_msgs
   DEPENDS Qt5Widgets Qt5Gui Qt5Core OpenCV
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(REMOTE-CONTROLLER-SRCS
  src/main.cpp
  src/pepper_speech.cpp
  src/pepper_movement.cpp
  src/pepper_animations.cpp
  src/pepper_leds.cpp
  src/pepper_activation.cpp
  src/remote_controller.cpp
  src/camera_thread.cpp
  src/js_controller.cpp
  src/controller_thread.cpp
  src/rviz_thread.cpp
)

set(REMOTE-CONTROLLER-HDRS
  include/remote_controller/pepper_speech.h
  include/remote_controller/pepper_animations.h
  include/remote_controller/pepper_leds.h
  include/remote_controller/pepper_activation.h
  include/remote_controller/remote_controller.h
  include/remote_controller/camera_thread.h
  include/remote_controller/pepper_movement.h
  include/remote_controller/js_controller.h
  include/remote_controller/controller_thread.h
  include/remote_controller/rviz_thread.h
)

# qt5_wrap_cpp(REMOTE-CONTROLLER-MOCS ${REMOTE-CONTROLLER-HDRS})

add_executable(remote_controller ${REMOTE-CONTROLLER-SRCS} ${REMOTE-CONTROLLER-HDRS})
target_link_libraries(remote_controller Qt5::Widgets ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# target_link_libraries(remote_controller PRIVATE Qt5::Widgets)
