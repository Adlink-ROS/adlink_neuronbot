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

## Developers & Team
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
* Realsense D400  
  Source: https://github.com/intel-ros/realsense  
  Notice: About RealSense SDK 2.0, we highly recommed binary version.  
  Testing: $ roslaunch realsense2_camera demo_pointcloud.launch  
  <br />
* Astra Pro OR Astra (alternative choose for camera)   
  Binary: $ sudo apt-get install ros-kinetic-astra*  
  Source: https://github.com/orbbec/ros_astra_camera  
  Notice: Remember to create udev. If possible, please buy Astra instead of Astra Pro!  
  <br />
* YDLidar   
  Source: https://github.com/EAIBOT/ydlidar  
  Notice: Remember to laod udev. Could be replaced by any type of lidar.  
  Testing: $ roslaunch ydlidar x4.launch  
  <br />
* Navigation  
  Binary: $ sudo apt-get install ros-kinetic-navigation*  
  Source: https://github.com/ros-planning/navigation  
  Notice: if "replan" mode of global planner is malfunctioned, please compile whole pkgs from source.  
  Testing: $ roslaunch spencer_people_tracking_launch tracking_on_bagfile.launch  
  <br />
* SPENCER  
  Binary: https://github.com/spencer-project/spencer_people_tracking#installation-from-l-cas-package-repository  
  Source: https://github.com/spencer-project/spencer_people_tracking#installation-from-source  
  Notice: Unless you want to use HOG+SVM [CUDA required], we highly recommend binary version.  
  <br />
* Intel object analytics  
  Source: https://github.com/intel/ros_object_analytics  
  Notice: Tested with NCSDK v1.12, ros_object_analytics should be "devel" branch  
  Testing:   
  ** movidius_ncs **  
     $ roslaunch realsense2_camera demo_pointcloud.launch  
     $ roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=mobilenetssd input_topic:=/camera/color/image_raw    
     $ roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:="/camera/color/image_raw"  
  ** object_analytics **  
     $ roslaunch realsense2_camera demo_pointcloud.launch  
     $ roslaunch object_analytics_launch analytics_movidius_ncs.launch input_points:=/camera/depth/color/points  
  <br />
* leg_tracker  
  Source: https://github.com/angusleigh/leg_tracker  
  Notice: The kinetic branch only supports OpenCv 3.3 and higher ver.  
  Testing: $ roslaunch leg_tracker demo_stationary_simple_environment.launch  
  <br />
* Turtlebot2  
  Binaty: $ sudo apt-get install ros-kinetic-turtlebot  
  Source: https://github.com/turtlebot/turtlebot  
  Notice: We highly recommed you to install binary version.  
  <br />

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

### Roadmap
* Fix PCL detector  
* ROS2 driver for camera  
* Fix followMe node (fused inputs version)  
* Create custom ROS2/DDS msg  
* Integrate with object_analytics  
 
