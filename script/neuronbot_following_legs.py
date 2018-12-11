#!/usr/bin/env python
# Copyright 2018 ADLINK Technology, Inc.
# Developer: HaoChih, LIN (haochih.lin@adlinktech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import sys
import time
import math
import tf
from geometry_msgs.msg import Twist, Pose2D, PoseStamped, PoseWithCovarianceStamped
from leg_tracker.msg import *
from dynamic_reconfigure.server import Server
from adlink_neuronbot.cfg import FollowingConfig
from kobuki_msgs.msg import BumperEvent, ButtonEvent

class FollowMe:
    def __init__(self, status):
        self.Ki_linear_window  = 0.95
        self.Ki_angular_window = 0.95
        self.linear_bound  = 0.17
        self.angular_bound = 1.2 #0.65
        self.target_id = -1
        self.status = status
        self.sum_error_r = 0
        self.sum_error_th = 0
        self.previous = 0
        self.first_dcb = True # To avoid Dynamic Server CB changing params at beginning
        self.listener = tf.TransformListener()
        self.bumper_flag = False # False: continue, True: stop
        self.debug = rospy.get_param('debug_info', True)
        self.following_distance = rospy.get_param('~following_distance', 0.9)
        self.dead_zone_radius   = rospy.get_param('~dead_zone_radius',  0.1) # +/- m
        self.dead_zone_theta    = rospy.get_param('~dead_zone_theta', 0.1) # +/- rad
        self.search_radius = rospy.get_param('~search_radius', 1.5) # +/- m
        self.search_theta  = rospy.get_param('~search_theta', 0.4) # +/- rad
        self.Kp_linear  = rospy.get_param('~Kp_linear', 0.8)
        self.Kp_angular = rospy.get_param('~Kp_angular', 1.2)
        self.Ki_linear  = rospy.get_param('~Ki_linear', 0.0)
        self.Ki_angular = rospy.get_param('~Ki_angular', 0.0)
        self.Ki_factor_linear  = rospy.get_param('~Ki_factor_linear',  0.95)
        self.Ki_factor_angular = rospy.get_param('~Ki_factor_angular', 0.95)
        self.base_frame = rospy.get_param('~base_frame', "/base_footprint")
        self.cmd_pub = rospy.Publisher('/following/cmd_vel', Twist, queue_size = 50)
        Server(FollowingConfig, self.dynamicCB)
        rospy.Subscriber('/following/legs', PersonArray, self.trackerCB)
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.BumperCB)
        rospy.Subscriber('/mobile_base/events/button', ButtonEvent, self.ButtonCB)

        # rosparam (for amcl init pose)
        self.amclPub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=2)
        self.initMsg = PoseWithCovarianceStamped()
        self.initMsg.header.frame_id = rospy.get_param('~map_frame', 'map')
        self.initMsg.pose.pose.position.x = rospy.get_param('~initPosX', 0.662)  # position
        self.initMsg.pose.pose.position.y = rospy.get_param('~initPosY', 0.427)  # position
        self.quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, rospy.get_param('~initPosA', 2.612))
        self.initMsg.pose.pose.orientation.z = self.quaternion[2]  # orientation
        self.initMsg.pose.pose.orientation.w = self.quaternion[3]  # orientation
        self.initMsg.pose.covariance[0]  = rospy.get_param('~initCovX', 0.25)  # covariance of x
        self.initMsg.pose.covariance[7]  = rospy.get_param('~initCovY', 0.25)  # covariance of y
        self.initMsg.pose.covariance[35] = rospy.get_param('~initCovA', 0.05)  # covariance of theta

    def BumperCB(self, msg):
        if msg.bumper >= 0: #left:0, center:1, right:2
            self.target_id = -1
            self.send_cmd(0.0, 0.0)
            rospy.loginfo('Bumper: Stop!')
            self.bumper_flag = True      

    def ButtonCB(self, msg):
        if msg.button == 0: #left:0, center:1, right:2
            # Re-initialize the pose of robot w.r.t the map
            self.initMsg.header.stamp = rospy.Time.now() 
            self.amclPub.publish(self.initMsg)            
            rospy.loginfo("Re-initial the pose of robot, x: %d, y: %d", self.initMsg.pose.pose.position.x, self.initMsg.pose.pose.position.y) 
            rospy.sleep(1.0) # wait 1 sec
        if msg.button >= 0: #left:0, center:1, right:2
            self.bumper_flag = False
            rospy.loginfo('Button: Release!')  

    def dynamicCB(self, config, level):
        # Update the default values of all dynamical params        
        if self.first_dcb:
            config.Status = self.status 
            config.Distance = self.following_distance
            config.Search_radius = self.search_radius
            config.Search_theta = self.search_theta
            config.Kp_linear = self.Kp_linear
            config.Kp_angular = self.Kp_angular
            config.Ki_linear = self.Ki_linear
            config.Ki_angular = self.Ki_angular
            config.DeadZone_radius = self.dead_zone_radius
            config.DeadZone_theta = self.dead_zone_theta
            self.first_dcb = False            
            return config

        self.status = config.Status
        self.following_distance = config.Distance
        self.search_radius = config.Search_radius
        self.search_theta  = config.Search_theta
        self.Kp_linear  = config.Kp_linear
        self.Kp_angular = config.Kp_angular
        self.Ki_linear  = config.Ki_linear
        self.Ki_angular = config.Ki_angular
        self.dead_zone_radius  = config.DeadZone_radius
        self.dead_zone_theta = config.DeadZone_theta

        if self.status == False:
            self.send_cmd(0.0, 0.0)
            rospy.loginfo('Stop following')
        if self.debug:
            rospy.loginfo('Dynamical Parameters Updated!')
        return config

    def trackerCB(self, msg):
        if self.status and self.bumper_flag == False:
            if self.target_id == -1:
                self.lock_target_id(msg)
            elif msg.people == []:
                self.target_id = -1
                rospy.loginfo('target lost')
                self.send_cmd(0.05, 0.0)
            else:
                r,th = self.get_target_position(msg)
                self.following(r,th)

    def xyToPolar(self, PersonArray, i):
        # transform PoseStamped from /odom to /base_link
        try:
            odomPerson = PoseStamped()
            basePerson = PoseStamped()
            odomPerson.header = PersonArray.header
            odomPerson.pose = PersonArray.people[i].pose
            basePerson = self.listener.transformPose(self.base_frame, odomPerson)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf listening failed')
            return -1, -1
        target_x = basePerson.pose.position.x
        target_y = basePerson.pose.position.y
        r = math.sqrt( math.pow(target_x, 2) + math.pow(target_y, 2) )
        th = math.atan2(target_y,target_x)
        return r,th

    def get_target_position(self, PersonArray):
        target_get = False
        for i in range(0,len(PersonArray.people)):
            if PersonArray.people[i].id == self.target_id:
                r,th = self.xyToPolar(PersonArray,i)
                target_get = True
                return r,th
        if not target_get:
            self.target_id = -1
            rospy.loginfo('target lost')
            self.send_cmd(0.05, 0.0)
            return -1, -1

    def lock_target_id(self, PersonArray):
        for i in range(0, len(PersonArray.people)):
            r,th = self.xyToPolar(PersonArray,i)
            if r <= self.search_radius and -self.search_theta <= th and th <= self.search_theta:
                self.target_id = PersonArray.people[i].id
                rospy.loginfo('target locked')

    def constrain(self, num, limit_upper, limit_lower):
        if num >= limit_upper:
            num = limit_upper
            return num
        elif num <= limit_lower:
            num = limit_lower
            return num
        else:
            return num

    def send_cmd(self, linear_x, angular_z):
        motion_cmd = Twist()
        motion_cmd.angular.z = angular_z
        motion_cmd.linear.x  = linear_x
        if self.bumper_flag == False:
            self.cmd_pub.publish(motion_cmd)

    def following(self, r, th):
        if r == -1:
            rospy.loginfo('target pose error !')
            return -1
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.previous
        # Calculate PI control
        error_r  = r - self.following_distance
        error_th = th - 0.0;
        if abs(error_r) >= self.dead_zone_radius:
            self.sum_error_r  = self.Ki_factor_linear*self.Ki_linear_window*self.sum_error_r + error_r*dt
            follow_speed_x = self.Kp_linear * error_r + self.Ki_linear  * (self.sum_error_r)
            follow_speed_x = self.constrain(follow_speed_x, self.linear_bound, -self.linear_bound)
        else:
            follow_speed_x = 0.0
            if self.debug:
                rospy.loginfo('Linear Dead Zone!')

        if abs(error_th) >= self.dead_zone_theta:
            self.sum_error_th = self.Ki_factor_angular*self.Ki_angular_window*self.sum_error_th + error_th*dt
            follow_speed_z = self.Kp_angular * (th)   + self.Ki_angular * (self.sum_error_th)
            follow_speed_z = self.constrain(follow_speed_z, self.angular_bound, -self.angular_bound)
        else:
            follow_speed_z = 0.0
            if self.debug:
                rospy.loginfo('Angular Dead Zone!')
        
        # Pub vel cmd
        self.send_cmd(follow_speed_x, follow_speed_z)
        self.previous = current_time
        if self.debug:
            rospy.loginfo('Target ID: ' + str(self.target_id) + ', r: ' + str(r) + ', th: ' + str(th))
            rospy.loginfo('PID linear: ' + str(follow_speed_x) + ', angular: ' + str(follow_speed_z))


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("Follow_me", anonymous = False)
    rospy.loginfo("===== neuronbot following node (legs) =====")
    rospy.loginfo("Waiting for leg_tracker msg ...")
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        fm = FollowMe(True)
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
