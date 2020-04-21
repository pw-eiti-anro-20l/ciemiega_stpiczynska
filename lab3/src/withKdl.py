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
    if not correct(data):
        rospy.logerr('Incorrect position! ' + str(data))
        return

    kdlChain = kdl.Chain()
    frameFactory = kdl.Frame();

    frame0 = frameFactory.DH(0, 0, 0, 0);
    joint0 = kdl.Joint(kdl.Joint.None)
    kdlChain.addSegment(kdl.Segment(joint0, frame0))

    a, d, al, th = params['i2']
    al, a, d, th = float(al), float(a), float(d), float(th)
    frame1 = frameFactory.DH(a, al, d, th)
    joint1 = kdl.Joint(kdl.Joint.RotZ)
    kdlChain.addSegment(kdl.Segment(joint1, frame1))

    a, d, al, th = params['i1']
    al, a, d, th = float(al), float(a), float(d), float(th)
    frame2 = frameFactory.DH(a, al, d, th)
    joint2 = kdl.Joint(kdl.Joint.RotZ)
    kdlChain.addSegment(kdl.Segment(joint2, frame2))

    a, d, al, th = params['i3']
    al, a, d, th = float(al), float(a), float(d), float(th)
    frame3 = frameFactory.DH(a, al, d, th)
    joint3 = kdl.Joint(kdl.Joint.TransZ)
    kdlChain.addSegment(kdl.Segment(joint3, frame3))

    jntAngles = kdl.JntArray(kdlChain.getNrOfJoints())
    jntAngles[0] = data.position[0]
    jntAngles[1] = data.position[1]
    jntAngles[2] = -data.position[2] - d

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
