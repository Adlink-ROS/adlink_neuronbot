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
import string
import math
import time
import sys

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray, GoalID
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from kobuki_msgs.msg import BumperEvent, ButtonEvent

class MultiGoals:
    def __init__(self, goalListX, goalListY, goalListZ, goalListW, retry, map_frame):
        self.goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=2) 
        self.stopPub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=2) 
        self.amclPub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=2)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.statusCB, queue_size=10) 
        rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.BumperCB)
        rospy.Subscriber('/mobile_base/events/button', ButtonEvent, self.ButtonCB)
        # rosparam (for amcl init pose)
        self.initMsg = PoseWithCovarianceStamped()
        self.initMsg.header.frame_id = map_frame
        self.initMsg.pose.pose.position.x = float( rospy.get_param('~initPosX', '0.0') ) # position
        self.initMsg.pose.pose.position.y = float( rospy.get_param('~initPosY', '0.0') ) # position
        self.initMsg.pose.pose.orientation.z = float( rospy.get_param('~initOriZ', '0.0') ) # orientation
        self.initMsg.pose.pose.orientation.w = float( rospy.get_param('~initOriW', '1.0') ) # orientation
        self.initMsg.pose.covariance[0]  = float( rospy.get_param('~initCovX', '0.25') ) # covariance of x
        self.initMsg.pose.covariance[7]  = float( rospy.get_param('~initCovY', '0.25') ) # covariance of y
        self.initMsg.pose.covariance[35] = float( rospy.get_param('~initCovA', '0.05') ) # covariance of theta
        # params & variables
        self.goalListX = goalListX
        self.goalListY = goalListY
        self.goalListZ = goalListZ
        self.goalListW = goalListW
        self.retry = retry
        self.status = False
        self.goalId = 0
        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = map_frame
        self.goalMsg.pose.orientation.z = 0.0
        self.goalMsg.pose.orientation.w = 1.0

    def BumperCB(self, msg):
        if msg.bumper >= 0: #left:0, center:1, right:2
            self.status = False
            emptyMsg = GoalID()                
            self.stopPub.publish(emptyMsg) 
            rospy.loginfo('Bumper: Stop!')     

    def ButtonCB(self, msg):
        if msg.button == 0 and msg.state == 0: #left:0, center:1, right:2
            # Re-initialize the pose of robot w.r.t the map
            self.initMsg.header.stamp = rospy.Time.now() 
            self.amclPub.publish(self.initMsg)            
            rospy.loginfo("Re-initial the pose of robot, x: %d, y: %d", self.initMsg.pose.pose.position.x, self.initMsg.pose.pose.position.y) 
            rospy.sleep(1.0) # wait 1 sec
            # Publish the first goal
            self.status = True
            self.goalId = 0
            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            self.goalPub.publish(self.goalMsg) 
            rospy.loginfo("Initial goal published! Goal ID is: %d", self.goalId) 
            self.goalId = self.goalId + 1 

        if msg.button == 1 and msg.state == 0: #left:0, center:1, right:2
            # Publish the continued goal
            if self.goalId == 0:
                self.goalId = (len(self.goalListX)-1)
            else:
                self.goalId = self.goalId - 1
            self.status = True
            self.goalMsg.header.stamp = rospy.Time.now()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            self.goalPub.publish(self.goalMsg) 
            rospy.loginfo("Next goal published! Goal ID is: %d", self.goalId) 
            if self.goalId < (len(self.goalListX)-1):
                self.goalId = self.goalId + 1
            else:
                self.goalId = 0 

    def statusCB(self, data):
        if data.status.status == 3 and self.status: # reached & status
            self.goalMsg.header.stamp = rospy.Time.now()                
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            self.goalPub.publish(self.goalMsg)  
            rospy.loginfo("Reached! Next goal published, goal ID is: %d", self.goalId)              
            if self.goalId < (len(self.goalListX)-1):
                self.goalId = self.goalId + 1
            else:
                self.goalId = 0 
                


if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('multi_goals', anonymous=True)

        # Get params
        goalListX = rospy.get_param('~goalListX', '[0.0, 1.0]') # position
        goalListY = rospy.get_param('~goalListY', '[0.0, 1.0]') # position
        goalListZ = rospy.get_param('~goalListZ', '[0.0, 0.0]') # orientation
        goalListW = rospy.get_param('~goalListW', '[1.0, 1.0]') # orientation
        map_frame = rospy.get_param('~map_frame', 'map' )
        retry = rospy.get_param('~retry', '1') 

        goalListX = goalListX.replace("[","").replace("]","")
        goalListY = goalListY.replace("[","").replace("]","")
        goalListZ = goalListZ.replace("[","").replace("]","")
        goalListW = goalListW.replace("[","").replace("]","")
        goalListX = [float(x) for x in goalListX.split(",")]
        goalListY = [float(y) for y in goalListY.split(",")]
        goalListZ = [float(z) for z in goalListZ.split(",")]
        goalListW = [float(w) for w in goalListW.split(",")]

        if len(goalListX) == len(goalListY) == len(goalListZ) == len(goalListW) and len(goalListX) >=2:          
            # Constract MultiGoals Obj
            rospy.loginfo("Multi Goals Executing...")
            mg = MultiGoals(goalListX, goalListY, goalListZ, goalListW, retry, map_frame)          
            rospy.spin()
        else:
            rospy.errinfo("Lengths of goal lists are not the same")
    except KeyboardInterrupt:
        print("shutting down")



