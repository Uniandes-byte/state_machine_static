## 			Vision Tools Message
string camera_name
string command 		#enable:  Enables the specified camera with default config
			#disable:  Disables the specified camera
			#custom:   Enables the specified camera with the configuration and parameters specified by the user
			#set_parameters: Set only the the parameters specified by the user
			#get_parameters: Get only the parameters specified by the user
	       
#Configurations
uint8 resolution
uint8 frame_rate
uint8 color_space

#Parameters

robot_toolkit_msgs/camera_parameters_msg camera_parameters


