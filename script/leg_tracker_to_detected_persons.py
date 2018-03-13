#!/usr/bin/python
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
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson
from geometry_msgs.msg import Pose
from leg_tracker.msg import PersonArray, Person

class Conversion:
    def __init__(self, detectionId, detectionIdIncrement, posVariance, rotVariance, confidence, modality):
        self.sub = rospy.Subscriber('/people_tracked', PersonArray, self.CallBack, queue_size=10)
        self.pub = rospy.Publisher('/leg_tracker/detected_persons', DetectedPersons, queue_size=10)   
        self.modality = modality
        self.confidence = confidence
        self.posVariance = posVariance
        self.rotVariance = rotVariance
        self.detectionId = detectionId
        self.detectionIdIncrement = detectionIdIncrement

    def CallBack(self, data):
        detectedPersons = DetectedPersons()
        detectedPersons.header = data.header                
        
        for Person in data.people:
            detectedPerson = DetectedPerson()
            detectedPerson.modality = self.modality
            detectedPerson.confidence = self.confidence
            detectedPerson.detection_id = self.detectionId
            detectedPerson.pose.pose = Person.pose
            
            for i in xrange(0, 6):
                detectedPerson.pose.covariance[i*6 + i] = self.posVariance if i < 3 else self.rotVariance
            
            detectedPersons.detections.append(detectedPerson)
            self.detectionId += self.detectionIdIncrement

        self.pub.publish(detectedPersons)
        # end of func


if __name__ == "__main__":
    try:    
        # ROS Init    
        rospy.init_node('leg_tracker_to_detected_persons', anonymous=True)

        # Get params
        detectionId = rospy.get_param("~detection_id_offset", 0)
        detectionIdIncrement = rospy.get_param("~detection_id_increment", 1)
        posVariance = rospy.get_param("~pos_variance", 0.1)
        rotVariance = rospy.get_param("~rot_variance", 99999)
        confidence  = rospy.get_param("~confidence", 1.0)
        modality    = rospy.get_param("~modality", "leg")

        # Constract Conversion Obj
        conv = Conversion(detectionId, detectionIdIncrement, posVariance, rotVariance, confidence, modality)
        rospy.loginfo("Start Conversion Node ...")
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
