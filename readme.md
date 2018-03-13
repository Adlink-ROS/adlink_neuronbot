# ADLINK NeuronBot

## Abstract  
The purpose of this pkg is to demonstrate two main features of ADLINK Neuron miniITX platform.   
1. Computing Power:   
   The host robot is able to follow human by integrating all outputs from SPENCER robot framework, intel "object analytics" pkg and laser based leg_tracker pkg.
   ADLINK Neuron illustates its computing power to run these detecting/tracking algorithms smoothly. (HOG+SVM, CNN, Template matching, EKF, MobileNetSSD, etc)  
2. ROS2/DDS Capabilities:  
   The host robot (for following) is publishing its own pose with respect to known map through ROS2/DDS layer while following a human.
   On the other hand, the client robot (random wandering), could avoid the host robot by receiving host's pose and replaning its path.   
   
[Official Slides] https://github.com/Adlink-ROS/adlink_neuronbot/blob/master/document/ADLINK_NeuronBot_20180313.pdf  
[Youtube Video] https://youtu.be/RC6XvTvTs9Y  
[Youtube Video] https://youtu.be/qA4_Hmnd_tM  
[![alt text](http://img.youtube.com/vi/RC6XvTvTs9Y/0.jpg)](https://youtu.be/RC6XvTvTs9Y)  

## Developer & Team
HaoChih Lin  
Alan Chen  
Chester Tseng  
Bill Wang  
Erik Boasson  
Ryan Chen  
  
ADLINK Technology, Inc  
Advanced Robotic Platform Group  

## License
Apache 2.0  
Copyright 2018 ADLINK Technology, Inc.  

## Tutorial
### System Prerequisite
[Packages]  
* SPENCER  
  Binary: https://github.com/spencer-project/spencer_people_tracking#installation-from-l-cas-package-repository  
  Compile: https://github.com/spencer-project/spencer_people_tracking#installation-from-source  
* Intel object analytics  
  https://github.com/intel/ros_object_analytics  
* Turtlebot2  
  https://github.com/turtlebot/turtlebot  
* Astra Pro:  
  https://github.com/orbbec/ros_astra_camera  
* YDLidar:  
  https://github.com/EAIBOT/ydlidar  

### Launching Steps
* Mapping & Time Synchronizing  
* Host robot (for following)  
  $ roslaunch adlink_neuronbot NeuronBot_Demo_Host_AIO.launch  
  OR (script)  
  $ ./PATH_TO_WORKSPACE/adlink_neuronbot/autostart/NeuronBot_Demo_Host_AutoStart.sh  
* Client robot (for avoidance)  
  $ roslaunch adlink_neuronbot NeuronBot_Demo_Client_AIO.launch  
  OR (script)  
  $ ./PATH_TO_WORKSPACE/adlink_neuronbot/autostart/NeuronBot_Demo_Client_AutoStart.sh  

### Known Issues
* Q: move_base replanning does not work  
  A: update your move_base pkg to the latest one, or compile it from source  
 
