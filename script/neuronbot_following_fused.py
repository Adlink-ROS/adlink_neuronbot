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
from std_msgs.msg import *
from geometry_msgs.msg import Twist, PoseStamped
from spencer_tracking_msgs.msg import TrackedPersons
from dynamic_reconfigure.server import Server
from adlink_neuronbot.cfg import FollowingConfig

class FollowMe:
    def __init__(self, status):
        self.Ki_linear_window  = 0.95
        self.Ki_angular_window = 0.95
        self.linear_bound  = 0.3
        self.angular_bound = 0.4
        self.target_id = -1
        self.status = status
        self.sum_error_r = 0
        self.sum_error_th = 0
        self.previous = 0
        self.first_dcb = True # To avoid Dynamic Server CB changing params at beginning
        self.listener = tf.TransformListener()
        self.debug = rospy.get_param('debug_info', True)
        self.following_distance = rospy.get_param('~following_distance', 0.9)
        self.dead_zone_radius   = rospy.get_param('~dead_zone_radius',  0.1) # +/- m
        self.dead_zone_theta    = rospy.get_param('~dead_zone_theta', 0.1) # +/- rad
        self.search_radius = rospy.get_param('~search_radius', 1.5) # +/- m
        self.search_theta  = rospy.get_param('~search_theta', 0.4) # +/- rad
        self.Kp_linear  = rospy.get_param('~Kp_linear', 0.8)
        self.Kp_angular = rospy.get_param('~Kp_angular', 1.5)
        self.Ki_linear  = rospy.get_param('~Ki_linear', 0.0)
        self.Ki_angular = rospy.get_param('~Ki_angular', 0.0)
        self.Ki_factor_linear  = rospy.get_param('~Ki_factor_linear',  0.95)
        self.Ki_factor_angular = rospy.get_param('~Ki_factor_angular', 0.95)
        self.base_frame = rospy.get_param('~base_frame', "/base_footprint")
        self.cmd_pub = rospy.Publisher('/following/cmd_vel', Twist, queue_size = 50)
        Server(FollowingConfig, self.dynamicCB)
        rospy.Subscriber('/following/mode', String, self.followCB)
        rospy.Subscriber('/following/persons', TrackedPersons, self.trackerCB)

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


    def followCB(self, String):
        if String.data == 'start':
            self.status = True
            rospy.loginfo('Start following')
        elif String.data == 'stop':
            self.status = False
            self.send_cmd(0.0, 0.0)
            rospy.loginfo('Stop following')
        else:
            rospy.logerr('Wrong type msg')

    def trackerCB(self, msg):
        if self.status:
            if self.target_id == -1:
                self.lock_target_id(msg)
            elif msg.tracks == []:
                self.target_id = -1
                rospy.loginfo('target lost')
                self.send_cmd(0.0, 0.0)
            else:
                r,th = self.get_target_position(msg)
                self.following(r,th)

    def xyToPolar(self, Persons, i):
        # transform PoseStamped from /odom to /base_link
        try:
            odomPerson = PoseStamped()
            basePerson = PoseStamped()
            odomPerson.header = Persons.header
            odomPerson.pose = Persons.tracks[i].pose.pose
            basePerson = self.listener.transformPose(self.base_frame, odomPerson)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('tf listening failed')
            return -1, -1
        target_x = basePerson.pose.position.x
        target_y = basePerson.pose.position.y
        r = math.sqrt( math.pow(target_x, 2) + math.pow(target_y, 2) )
        th = math.atan2(target_y,target_x)
        return r,th

    def get_target_position(self, Persons):
        target_get = False
        for i in range(0,len(Persons.tracks)):
            if Persons.tracks[i].track_id == self.target_id and Persons.tracks[i].is_matched == True:
                r,th = self.xyToPolar(Persons, i)
                target_get = True
                return r,th
        if not target_get:
            self.target_id = -1
            rospy.loginfo('target lost')
            self.send_cmd(0.0, 0.0)
            return -1, -1

    def lock_target_id(self, Persons):
        for i in range(0, len(Persons.tracks)):
            r,th = self.xyToPolar(Persons, i)
            if r <= self.search_radius and -self.search_theta <= th and th <= self.search_theta:
                self.target_id = Persons.tracks[i].track_id
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
    rospy.loginfo("===== neuronbot following node (fused) =====")
    rospy.loginfo("Waiting for TrackedPersons msg ...")
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        fm = FollowMe(True)
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
