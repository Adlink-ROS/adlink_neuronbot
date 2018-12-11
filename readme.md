# ADLINK NeuronBot
V3.0-NCS is a AI vision people tracker. It needs Intel Realsense D435 + Movidius NCS 1.

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
Ewing Kang  
Alan Chen  
Chester Tseng  
Bill Wang  
Erik Boasson  
HaoChih Lin  
Ryan Chen  
  
ADLINK Technology, Inc  
Advanced Robotic Platform Group  

## License
Apache 2.0  
Copyright 2018 ADLINK Technology, Inc.  

## Tutorial
### System Prerequisite (Packages)  
* Intel Realsense SDK 2.0  
  Source: https://github.com/IntelRealSense/librealsense/releases/tag/v2.16.3  
  Version: SDK 2.0 (2.16.3)  
  Notice: Apt install is recommanded  
  
* Realsense D435 firmware 5.10.3
  Link: [firmware](https://downloadcenter.intel.com/download/28237/Latest-Firmware-for-Intel-RealSense-D400-Product-Family?v=t) / [tools and manual](https://www.intel.com/content/www/us/en/support/articles/000028171/emerging-technologies/intel-realsense-technology.html)
  
* Realsense ROS wrapper 2.0 (2.1.1)  
  Link: https://github.com/intel-ros/realsense/releases

* Intel object analytics  
  Source: https://github.com/intel/ros_intel_movidius_ncs/tree/devel   
  Install:  
    1. Install the [NCSDK](https://github.com/movidius/ncsdk) v1.12.01
        make sure you have environment set, **without ros stuff**
    2. Clone and build [NC APP Zoo](https://github.com/movidius/ncappzoo) to /opt/movidius directly
        * `make all` will fail due to (a) v4l unable to use realsense as video0 
        * Remove ../tensorflow/topcoder_maups that causes build failure
        * ` make compile'
    3. Install movidius ncs ROS package with branch "devel"  
    
  Testing:  
    * movidius_ncs  
     `$ roslaunch realsense2_camera demo_pointcloud.launch`  
     `$ roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=mobilenetssd input_topic:=/camera/color/image_raw`  
     `$ roslaunch movidius_ncs_launch ncs_stream_detection_example.launch camera_topic:="/camera/color/image_raw"`  
    * object_analytics  
     `$ roslaunch realsense2_camera demo_pointcloud.launch`  
     `$ roslaunch object_analytics_launch analytics_movidius_ncs.launch input_points:=/camera/depth/color/points`  
    * You can modify _movidius_ncs_launch/ncs_camera.launch_, so that realsense package can activate properly `realsense_ros_camera -> realsense2_camera`
    * **When in doubt, unplug and replug everything**  
  

  
* Astra Pro OR Astra (alternative choose for camera)   
  Binary: `$ sudo apt-get install ros-kinetic-astra*`  
  Source: https://github.com/orbbec/ros_astra_camera  
  Notice: Remember to create udev. If possible, please buy Astra instead of Astra Pro!  
* YDLidar   
  Source: https://github.com/EAIBOT/ydlidar  
  Notice: Remember to laod udev. Could be replaced by any type of lidar.  
  Testing: `$ roslaunch ydlidar x4.launch`  
* Navigation  
  Binary: `$ sudo apt-get install ros-kinetic-navigation*`  
  Source: https://github.com/ros-planning/navigation  
  Notice: if "replan" mode of global planner is malfunctioned, please compile whole pkgs from source.  
  Testing: `$ roslaunch spencer_people_tracking_launch tracking_on_bagfile.launch`
* SPENCER  
  Binary: https://github.com/spencer-project/spencer_people_tracking#installation-from-l-cas-package-repository  
  Source: https://github.com/spencer-project/spencer_people_tracking#installation-from-source  
  Notice: Unless you want to use HOG+SVM [CUDA required], we highly recommend binary version.  
* leg_tracker  
  Source: https://github.com/angusleigh/leg_tracker  
  Notice: The kinetic branch only supports OpenCv 3.3 and higher ver.  
  Testing: `$ roslaunch leg_tracker demo_stationary_simple_environment.launch`  
* Turtlebot2  
  Binaty: `$ sudo apt-get install ros-kinetic-turtlebot`  
  Source: https://github.com/turtlebot/turtlebot  
  Notice: We highly recommed you to install binary version.  

### Launching Steps
* Mapping & Time Synchronizing  
  `$ roslaunch adlink_neuronbot 
* Host robot AI following  
  `$ roslaunch adlink_neuronbot NeuronBot_Demo_Host_Gmapping.launch`  
* Host robot AI following shell script  
  `$ ./PATH_TO_WORKSPACE/adlink_neuronbot/autostart/NeuronBot_Demo_Host_AutoStart.sh`  
* **Deprecated** Client robot (for avoidance)  
  `$ roslaunch adlink_neuronbot NeuronBot_Demo_Client_AIO.launch`  
  OR (script)  
  `$ ./PATH_TO_WORKSPACE/adlink_neuronbot/autostart/NeuronBot_Demo_Client_AutoStart.sh`  

### Known Issues
* Q: Errors and nothing happends after boot  
  A: Due to large power draw from lidar and Realsense, Movidius sometimes doesn't work at the first try. Replug Realsense and Movidius and try again  

### Roadmap
* Lidar fused following version  
* ROS2 migration
 
