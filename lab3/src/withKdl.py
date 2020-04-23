#! /usr/bin/python

import rospy
import json
import os
import PyKDL as kdl
import math
from tf.transformations import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)


def correct(pose):
    if pose.position[0] < rest['i1'][0] or pose.position[0] > rest['i1'][1]:
        return False

    if pose.position[1] < rest['i2'][0] or pose.position[1] > rest['i2'][1]:
        return False

    if pose.position[2] < rest['i3'][0] or pose.position[2] > rest['i3'][1]:
        return False

    return True


def forward_kinematics(data):
    doPub = True
    if not correct(data):
        rospy.logerr('Incorrect position! ' + str(data))
        doPub = False
        return


    kdlChain = kdl.Chain()
    frameFactory = kdl.Frame()
    jntAngles = kdl.JntArray(3)
    jointNums = [2,3,1]

    for i in jointNums:

        a, d, al, th = params['i'+str(i)]
        al, a, d, th = float(al), float(a), float(d), float(th)
        frame = frameFactory.DH(a, al, d, th)
        joint = kdl.Joint(kdl.Joint.RotZ)
        kdlChain.addSegment(kdl.Segment(joint, frame))

        jntAngles[i-1] = data.position[i-1]
        fksolver = kdl.ChainFkSolverPos_recursive(kdlChain)
        eeFrame = kdl.Frame()
        fksolver.JntToCart(jntAngles, eeFrame)
        quaternion = eeFrame.M.GetQuaternion()

        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = eeFrame.p[0]
        pose.pose.position.y = eeFrame.p[1]
        pose.pose.position.z = eeFrame.p[2]
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        if doPub:
            pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('KDL_KIN', anonymous=True)

    pub = rospy.Publisher('KdlAxes', PoseStamped, queue_size=10)
 

    rospy.Subscriber('joint_states', JointState, forward_kinematics)

    params = {}
    print os.path.dirname(os.path.realpath(__file__))
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../dh.json', 'r') as file:
        params = json.loads(file.read())

    rest = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../rest.json', 'r') as file:
        rest = json.loads(file.read())

    rospy.spin()
