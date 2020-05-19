#!/usr/bin/env python

import json
from collections import OrderedDict
import os
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from math import atan2, sqrt, pi


def inverse_kinematics(data):
    global a2
    global a3
    global current_theta
    global rest

    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

    #if x**2 + y**2 <= 0.:
     #   rospy.logerr('Warning. Position infinite.')
      #  return

    theta = [None]*3

    c3 = (x**2 + y**2 + z**2 - a2**2 - a3**2)/(2*a2*a3)
    try:
        s3 = sqrt(1 - c3**2)
    except ValueError:
        rospy.logerr('Warning. Wrong position.')
        return

    check = False


    if abs(atan2(y, x) - current_theta[0]) < abs((atan2(-y, -x)) - current_theta[0]):
        theta[0] = atan2(y, x)
        check = True
    else:
        theta[0] = atan2(-y, -x)

   
    if abs(atan2(s3, c3) - current_theta[2]) < abs(atan2(-s3, c3) - current_theta[2]):
        theta[2] = atan2(s3, c3)
        if check:
            theta[1] = atan2(-z, sqrt(x**2 + y**2)) - atan2(a3*s3, a2 + a3*c3)
        else:
            theta[1] = pi - atan2(-z, sqrt(x**2 + y**2)) + atan2(a3*s3, a2 + a3*c3)
    else:
        theta[2] = atan2(-s3, c3)
        if check:
            theta[1] = atan2(-z, sqrt(x**2 + y**2)) - atan2(-a3*s3, a2 + a3*c3)
        else:
            theta[1] = pi - atan2(-z, sqrt(x**2 + y**2)) + atan2(-a3*s3, a2 + a3*c3)

    #rospy.logerr(theta)

    current_theta = theta
    jointState = JointState()
    jointState.header = Header()
    jointState.header.stamp = rospy.Time.now()
    jointState.name = ['base_link_to_link1', 'joint2', 'joint3']
    jointState.position = theta
    jointState.velocity = []
    jointState.effort = []

    pub = rospy.Publisher('jint_control', JointState, queue_size=10)
    pub.publish(jointState)


if __name__ == "__main__":

    a2 = 1
    a3=0.2
    current_theta = [0.0]*3

    rospy.init_node('ikin', anonymous=True)
    rospy.Subscriber('oint_pose', PoseStamped, inverse_kinematics)
    rospy.spin()
