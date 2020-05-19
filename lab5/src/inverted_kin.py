#!/usr/bin/env python

import json
from collections import OrderedDict
import os
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from math import atan2, sqrt, pi

def getParams():
    path = os.path.realpath(__file__)
    with open(os.path.dirname(path) + '/../param_files/dh_file.json') as input_file:
        params = json.loads(input_file.read(), object_pairs_hook=OrderedDict)
    return params

def getRestrictions():
    path = os.path.realpath(__file__)
    with open(os.path.dirname(path) + '/../restrictions.json') as input_file:
        rest = json.loads(input_file.read(), object_pairs_hook=OrderedDict) 
    return rest

def checkRestrictions(theta, rest):
    for i in range(1,4):
        if theta[i-1] < rest["i" + str(i)][0] or theta[i-1] > rest["i" + str(i)][1]:
            return False
    return True

def fill_jointState(theta):
    jointState = JointState()
    jointState.header = Header()
    jointState.header.stamp = rospy.Time.now()
    jointState.name = ['base_link_to_link1', 'joint2', 'joint3']
    jointState.position = theta
    jointState.velocity = []
    jointState.effort = []
    return jointState

def inverse_kinematics(data):
    global a2
    global a3
    global current_theta
    global rest

    px = data.pose.position.x
    py = data.pose.position.y
    pz = data.pose.position.z

    if px**2 + py**2 <= 0.:
        rospy.logerr('Infinite solutions')
        return

    theta = [None]*3

    c3 = (px**2 + py**2 + pz**2 - a2**2 - a3**2)/(2*a2*a3)
    try:
        s3 = sqrt(1 - c3**2)
    except ValueError:
        rospy.logerr('Position unreachable')
        return

    flag = False

    #Joint_1 (base_to_link_1)
    if abs(atan2(py, px) - current_theta[0]) < abs((atan2(-py, -px)) - current_theta[0]):
        theta[0] = atan2(py, px)
        flag = True
    else:
        theta[0] = atan2(-py, -px)

    #Joint_2 (link1_to_link_2) and Joint_3 (link2_to_link_3)
    if abs(atan2(s3, c3) - current_theta[2]) < abs(atan2(-s3, c3) - current_theta[2]):
        theta[2] = atan2(s3, c3)
        if flag:
            theta[1] = atan2(-pz, sqrt(px**2 + py**2)) - atan2(a3*s3, a2 + a3*c3)
        else:
            theta[1] = pi - atan2(-pz, sqrt(px**2 + py**2)) + atan2(a3*s3, a2 + a3*c3)
    else:
        theta[2] = atan2(-s3, c3)
        if flag:
            theta[1] = atan2(-pz, sqrt(px**2 + py**2)) - atan2(-a3*s3, a2 + a3*c3)
        else:
            theta[1] = pi - atan2(-pz, sqrt(px**2 + py**2)) + atan2(-a3*s3, a2 + a3*c3)

    #if not checkRestrictions(theta, rest):
     #   rospy.logerr('Restrictions violation')
      #  return

    current_theta = theta
    jointState = fill_jointState(theta)

    pub = rospy.Publisher('jint_control', JointState, queue_size=10)
    pub.publish(jointState)

def ikin():
    rospy.init_node('ikin', anonymous=True)
    rospy.Subscriber('oint_pose', PoseStamped, inverse_kinematics)
    rospy.spin()

if __name__ == "__main__":
    #params = getParams()
    #rest = getRestrictions()
    #a2 = params['i3'][0]
    a2=1
    a3 = 0.2
    current_theta = [0.0]*3

    try:
        ikin()
    except rospy.ROSInterruptException:
        pass