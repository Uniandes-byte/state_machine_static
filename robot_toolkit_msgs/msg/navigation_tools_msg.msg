## 			Navigation Tools Message
string command #enable_all:  Enables the navigation tools functionalities with default frequencies
	       #disable_all: Disables the navigation tools functionalities
	       #custom:  Enables or disables the navigation tools functionalities individually according with the following fields
#tf topic
bool tf_enable
float32 tf_frequency

#odom topic
bool odom_enable
float32 odom_frequency

#laser topic
bool laser_enable
float32 laser_frequency

#cmd_vel subscriber 
bool cmd_vel_enable
float32 security_timer		#Security time used to stop the robot when there are no new messages in the cmd_topic, -1 to disable the security timer

#move base
bool move_base_enable

#navigation goal suscriber
bool goal_enable

#robot pose suscriber
bool robot_pose_suscriber_enable

#path publisher
bool path_enable
float32 path_frequency

#robot pose publisher
bool robot_pose_publisher_enable
float32 robot_pose_publisher_frequency

#result publisher
bool result_enable

#depth to laser
bool depth_to_laser_enable
robot_toolkit_msgs/depth_to_laser_msg depth_to_laser_parameters

#free zone
bool free_zone_enable




