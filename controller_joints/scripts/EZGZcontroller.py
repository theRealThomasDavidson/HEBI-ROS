#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import JointState, Joy
from visualization_msgs.msg import Marker, MarkerArray
from hebiros.srv import AddGroupFromNamesSrv
#include "hebiros/AddGroupFromNamesSrv.h"

import scratch
import numpy as np

def talker(state, pubJS, pubMRK, F, J):
    rate = rospy.Rate(60) # 60hz
    while not rospy.is_shutdown():
        if not mode[1]:
            currx, curry = tuple([x*.65 for x in inputs["right stick"]])
            currx *= -1
            delx,  dely = currx - state[0], currx - state[1]
        else:
            delx, dely = tuple([-x*.03 for x in inputs["right stick"]])
            #dely *= -1
            temp = F(state)
            currx = max(min(temp[0] + delx, 0.65), -0.65)
            curry = max(min(temp[1] + dely, 0.65), -0.65)
        x = currx
        y = curry
        if (currx ** 2 + curry ** 2) > .4225:
            a = (.39 ** .5)/((currx ** 2 + curry ** 2)**.5)
            x *= a
            y *= a

        if not mode[0] and (currx ** 2 + curry ** 2) > .02:

            if not mode[1]:
                th, ph = scratch.newtonMethod(state, F, .001, J, (x, -y))
            else:
                temp = F(state)
                th, ph = scratch.LinearSlide(state, F, J, (x, y))
                curry *= -1
            state = th, ph
        else:
            th, ph = state
        head = Header()
        head.stamp = rospy.Time.now()
        head.frame_id = "/base"
        msgMRKA = MarkerArray()
        msgMRK = Marker()
        msgMRK.header = head
        msgMRK.ns = "basic_shapes"
        msgMRK.id = 0
        msgMRK.type = Marker.SPHERE
        msgMRK.action = Marker.ADD

        msgMRK.pose.position.x = currx
        msgMRK.pose.position.y = curry
        msgMRK.pose.position.z = .4
        msgMRK.pose.orientation.x = 0.0
        msgMRK.pose.orientation.y = 0.0
        msgMRK.pose.orientation.z = 0.0
        msgMRK.pose.orientation.w = 1.0
        msgMRK.scale.x = .05
        msgMRK.scale.y = .05
        msgMRK.scale.z = .05
        msgMRK.color.r = 1.0
        msgMRK.color.g = 0.0
        msgMRK.color.b = 0.0
        msgMRK.color.a = 1.0
        msgMRK.lifetime.secs = 0
        msgMRK.lifetime.nsecs = int((1/60) * 10**9)

        msgtopub = JointState()
        msgtopub.header = head
        msgtopub.name = ["/HEBI/base", "/HEBI/shoulder", "/HEBI/elbow", "/HEBI/wrist"]
        msgtopub.position = [th, 0, ph, 0]
        msgtopub.effort = []
        msgtopub.velocity = []
        msgMRKA.markers = [msgMRK]
        pubMRK.publish(msgMRK)
        pubJS.publish(msgtopub)
        rate.sleep()

def callbackJoy(joyMsg, state):
    #if inputs["X"] == 0 and joyMsg.buttons[3] == 1:
        #mode[2] = not mode[2]
    if inputs["B"] == 0 and joyMsg.buttons[1] == 1:
        mode[1] = not mode[1]
    if inputs["A"] == 0 and joyMsg.buttons[0] == 1:
        mode[0] = not mode[0]

    inputs["time"] = joyMsg.header.stamp
    inputs["right stick"] = tuple(joyMsg.axes[0:2])
    inputs["A"] = joyMsg.buttons[0]
    inputs["X"] = joyMsg.buttons[3]
    inputs["B"] = joyMsg.buttons[1]
    state = inputs["right stick"]


def callbackFeedback(jointStateMsg):
    feedback = jointStateMsg


if __name__ == '__main__':
    global inputs
    inputs = {"time": 0.,
              "right stick": (0., 0.),
              "A": 0,
              "B": 0,
              "X": 0,
              }
    state = [0., 0.]
    global mode
    mode = [False, False, False]
    #odomSub = rospy.Subscriber("/turtlebot/mobile_base/odom", Odometry, callbackOdom)
    #smoothedBallSub = rospy.Subscriber("/colored_ball/pose3D", PoseStamped, callbackSmBl)

    F, J, E = scratch.newtonArgs()
    rospy.init_node('controller_listen', anonymous=True)
    global group_name
    group_name = "MyArm"
    rospy.wait_for_service("/hebiros/add_group_from_names")

    add_group = rospy.ServiceProxy("/hebiros/add_group_from_names", AddGroupFromNamesSrv)

    feedback_subscriber = rospy.Subscriber("/hebiros/" + group_name + "/feedback/joint_state", JointState,
                                           callback=callbackFeedback)
    pubGZJS = rospy.Publisher("/hebiros/" + group_name + "/command/joint_state", JointState, queue_size=100)

    names = {"base", "shoulder", "elbow", "wrist"}
    families = {"HEBI"}
    resp1 = None
    while not resp1:
        resp1 = add_group(group_name, names, families)


    rospy.Subscriber('/joy', Joy, callback=callbackJoy, callback_args=state)

    pubJS = rospy.Publisher('/joint_states', JointState, queue_size=10)
    pubMRK = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    try:
        talker(state, pubGZJS, pubMRK, F, J)
    except rospy.ROSInterruptException:
        pass
