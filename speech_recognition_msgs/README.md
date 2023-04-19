# speech_utilities

## Execution
When roscore is available run:

 ### Node: saveAudio_node
 
 *Description: This node enables the save_audio_srv.
 
 *Call node example:
  
    rosrun speech_recognition_pkg save_recording_seconds.py

 ### Node: speech_utilities
 
 *Description: This node enables the s2t_google_srv, s2t_vosk_srv, talk_speech_srv
  
    rosrun speech_recognition_pkg speech_utilities.py

### Node: Q_A_Node
 
 *Description: This node enables the q_a_speech_srv
  
    rosrun speech_recognition_pkg q_a_speech.py


## Services

speech_recognition_pkg offers the following services:

### save_audio_srv
  *Description: This service allows to record an audio with the microphones of the robot with a duration of x seconds. 
  
  *Service file: saveAudio_srv.srv
  
    Request:
       secons (int64): Indicates the duration for recording the audio.  
    Response:
       answer (string): Indicates if the audio file was created. 

    Call service example:

rosservice call /speech_recognition_pkg/save_audio_srv 8




speech_utilities offers the following services:

### s2t_vosk_srv
  *Description: This service allows to convert the audio saved previously on text via a remote model called Vosk. 
  
  *Service file: s2t_vosk_srv.srv
  
    Request:
       none 
    Response:
       answer (string): Indicates the converted audio in text. 

    Call service example:

rosservice call /speech_utilities/s2t_vosk_srv 

### s2t_google_srv

  *Description: This service allows to convert the audio saved previously on text via a google cloud service model. 
  
  *Service file: s2t_google_srv.srv
  
    Request:
       none 
    Response:
       answer (string): Indicates the converted audio in text. 

    Call service example:

rosservice call /speech_utilities/s2t_google_srv 

### talk_speech_srv

  *Description: This service allows the robot to say the input of the service. 
  
  *Service file: talk_speech_srv.srv
  
    Request:
       key (string): Indicates the phrase that the robot must say.  
    Response:
       answer (string): Indicates the converted audio in text. 

    Call service example:

rosservice call /speech_utilities/talk_speech_srv 'Hi my name is Nova' 

### q_a_speech_srv

  *Description: This service allows the robot to say some questions pre established, start reconding the audio throw the save_audio_srv and return an answer with the s2t_vosk_srv and s2t_google_srv (AÚN NO ESTÁ TERMINADO TODO LO ÚLTIMO, EL ENSAMBLE GENERAL, FUNCIONA HASTA LA PARTE DE GUARDAR EL AUDIO). 
  
  *Service file: q_a_speech.srv
  
    Request:
       key (string): Indicates the key word for the question that the robot will say. For example: 'birth' if for 'When is your birthday?'. Allowed keys: name, age, birth, drink, need, look, follow, navigate. 
    Response:
       answer (string): Indicates what Pepper ask for (the question).  

    Call service example:

rosservice call /speech_utilities/q_a_speech_srv age 

