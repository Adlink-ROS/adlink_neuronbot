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

class FollowMe:
    def __init__(self):
        self.quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, rospy.get_param('~yaw', 2.612))

    def conversion(self):
        print self.quaternion
        print self.quaternion[2]
        print self.quaternion[3]


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node("Follow_me", anonymous = False)
    rospy.loginfo("===== Conversion Test =====")

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        fm = FollowMe()
        fm.conversion()
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
