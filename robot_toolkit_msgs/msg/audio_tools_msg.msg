## 			Vision Tools Message
string command 		#enable:   		    Enables mic streaming and tts
			#disable:  		    Disables the mic streaming and tts
			#enable_mic: 		    Enables mic streaming 
			#disable_mic: 		    Disables the mic streaming
		        #custom:     		    Enables the mic streaming with the configuration specified by the user
			#enable_tts: 		    Enables tts
			#disable_tts: 		    Disables tts
			#enable_localization:       Enables audio localization
			#disable_localization:      Disables audio localization
			#get_speech_params:	    Gets the current speech params
			#set_speech_params:	    Sets the speech params
			#reset_speech_params:	    Resets Speech params to default
			
uint16 frequency	#16000Hz or 48000Hz
uint8 channels		# 0 = ALLCHANNELS
			# 1 = LEFTCHANNEL 
			# 2 = RIGHTCHANNEL
			# 3 = FRONTCHANNEL
			# 4 = REARCHANNEL

robot_toolkit_msgs/speech_parameters_msg speech_parameters
			
		
			