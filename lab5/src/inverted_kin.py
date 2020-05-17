#!/usr/bin/env python

import rospy
import json
import os
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from math import *
from lab5.srv import oint_control



def inverted_kinematics(data):
    global a1
    global a2
    global d



   # current_pos = rospy.wait_for_message('joint_states', JointState, timeout = 10).position

    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

    dz = sqrt(x**2 + y**2)

    alfa1 = atan2(y, x)

    try:
        alfa2 = acos((-a2**2 + a1**2 + dz**2) / (2 * a1 * dz))
        alfa3 = acos((a1**2 + a2**2 - dz**2) / (2 * a1 * a2))
    except:
        rospy.logerr('warning')
        return

    theta1 = (alfa1 + alfa2, alfa1 - alfa2)
    theta2 = (-pi + alfa3, pi - alfa3)
    theta3 = -z

    th1 = theta1[0]
    th2 = theta2[0]

    computed_joint_state = JointState()
    computed_joint_state.header = Header()
    computed_joint_state.header.stamp = rospy.Time.now()
    computed_joint_state.name = ['base_to_link1', 'link1_to_link2', 'link2_to_link3']

    computed_joint_state.position = [th1, th2, theta3]
    computed_joint_state.velocity = []
    computed_joint_state.effort = []
    jstate_pub.publish(computed_joint_state)

if __name__ == "__main__":
    params = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../dh.json', 'r') as file:
        params = json.loads(file.read())
    rest = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../rest.json', 'r') as file:
        rest = json.loads(file.read())

    a1 = float(params['i2'][0])
    a2 = float(params['i3'][0])
    d = float(params['i3'][1])

    rospy.init_node('ikin_node')
    sub = rospy.Subscriber('oint_pose', PoseStamped, inverted_kinematics)
    jstate_pub = rospy.Publisher('jstate', JointState, queue_size=10)
    rospy.spin()
