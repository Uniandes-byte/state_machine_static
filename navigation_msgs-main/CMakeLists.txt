cmake_minimum_required(VERSION 2.8.3)
project(navigation_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  simple_feedback_msg.msg
)

add_service_files(
  FILES
  add_place_srv.srv
  go_to_place_srv.srv
  go_to_relative_point_srv.srv
  robot_stop_srv.srv
  set_current_place_srv.srv
  start_random_navigation_srv.srv
  spin_srv.srv
  follow_you_srv.srv
  go_to_defined_angle_srv.srv
  get_absolute_position_srv.srv
  get_route_guidance_srv.srv
  correct_position_srv.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS geometry_msgs message_runtime
)

include_directories( include
   ${catkin_INDLUCE_DIRS}
)
