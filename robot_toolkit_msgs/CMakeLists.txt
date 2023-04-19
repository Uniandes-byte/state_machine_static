cmake_minimum_required(VERSION 2.8.3)
project(robot_toolkit_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
  navigation_tools_msg.msg
  depth_to_laser_msg.msg
  vision_tools_msg.msg
  camera_parameters_msg.msg
  audio_tools_msg.msg
  speech_msg.msg
  speech_parameters_msg.msg
  audio_localization_msg.msg
  motion_tools_msg.msg
  animation_msg.msg
  set_angles_msg.msg
  misc_tools_msg.msg
  leds_parameters_msg.msg
  face_detection_msg.msg
  path_msg.msg
  special_settings_msg.msg
  touch_msg.msg
  )

add_service_files(
  FILES
  navigation_tools_srv.srv
  vision_tools_srv.srv
  audio_tools_srv.srv
  motion_tools_srv.srv
  misc_tools_srv.srv
  speech_recognition_srv.srv
  )
  
generate_messages(DEPENDENCIES std_msgs  sensor_msgs)

catkin_package( CATKIN_DEPENDS message_runtime std_msgs )



