#! /usr/bin/python

import rospy
import json
import os
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


def kinematics(data):
    if not correct(data):
        rospy.logerr('Incorrect position! ' + str(data))
        return

    a, d, al, th = params['i1']
    al, a, d, th = float(al), float(a), float(d), float(th)
    tz = translation_matrix((0, 0, d))
    rz = rotation_matrix(data.position[0], zaxis)
    tx = translation_matrix((a, 0, 0))
    rx = rotation_matrix(al, xaxis)
    T1 = concatenate_matrices(rx, tx, rz, tz)

    a, d, al, th = params['i2']
    al, a, d, th = float(al), float(a), float(d), float(th)
    tz = translation_matrix((0, 0, d))
    rz = rotation_matrix(data.position[1], zaxis)
    tx = translation_matrix((a, 0, 0))
    rx = rotation_matrix(al, xaxis)
    T2 = concatenate_matrices(rx, tx, rz, tz)

    a, d, al, th = params['i3']
    al, a, d, th = float(al), float(a), float(d), float(th)
    tz = translation_matrix((0, 0, d))
    rz = rotation_matrix(data.position[2], zaxis)
    tx = translation_matrix((a, 0, 0))
    rx = rotation_matrix(al, xaxis)
    T3 = concatenate_matrices(rx, tx, rz, tz)

    Tk = concatenate_matrices(T1, T2, T3)
    x, y, z = translation_from_matrix(Tk)
    q_x, q_y, q_z, q_w = quaternion_from_matrix(Tk)

    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    pub.publish(pose)



if __name__ == '__main__':
    rospy.init_node('NONKDL_KIN', anonymous=False)

    pub = rospy.Publisher('NoKdlAxes', PoseStamped, queue_size=10)

    rospy.Subscriber('joint_states', JointState, kinematics)

    params = {}
    print os.path.dirname(os.path.realpath(__file__))
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../dh.json', 'r') as file:
        params = json.loads(file.read())

    rest = {}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../rest.json', 'r') as file:
        rest = json.loads(file.read())

    rospy.spin()
