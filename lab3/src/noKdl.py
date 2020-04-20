#! /usr/bin/python

import json
import rospy
import os
from sensor_msgs.msg import JointState
from tf.transformations import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped


def isPositionAvailable(data):
    i=0
    for instance in restrictionsJson:
        oneInstance= json.loads(json.dumps(instance))
        if data.position[i]>oneInstance["forward"] or data.position[i]<oneInstance["backward"] :
            return False
        i=i+1
    return True
def callback(data):
    if isPositionAvailable(data)==False:
        rospy.logerr("Position is not available: " + str(data))
        return
    T = translation_matrix((0, 0, 0));
    
    i=0
    for instance in dhJson:
        oneInstance= json.loads(json.dumps(instance))
        a = oneInstance["a"]
        d = oneInstance["d"]
        alpha=oneInstance["alpha"]
        theta = oneInstance["theta"]

        matrixD= translation_matrix((0, 0, d*(1+data.position[i])))
        matrixTheta = rotation_matrix(theta, zaxis)
        matrixA = translation_matrix((a, 0, 0))
        matrixAlpha = rotation_matrix(alpha, xaxis)

        TransMatrix = concatenate_matrices(matrixA,matrixAlpha,matrixTheta, matrixD)
        T=concatenate_matrices(T, TransMatrix)
        i=i+1
        
    
    x, y, z = translation_from_matrix(T)
    qx, qy, qz, qw = quaternion_from_matrix(T)
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()
    
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    pub.publish(pose)
    


if __name__ == '__main__':
    xaxis, yaxis, zaxis = (1, 0, 0), (0, 1, 0), (0, 0, 1)
    rospy.init_node('NONKDL_DKIN', anonymous=False)
    dhJson ={}
    restrictionsJson ={}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../restrictions.json', 'r') as file:
        restrictionsJson= json.loads(file.read())
   
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../dh_data.json', 'r') as file:
        dhJson= json.loads(file.read())

    pub = rospy.Publisher('NoKdlAxes', PoseStamped, queue_size=10)
    rospy.Subscriber("joint_states", JointState , callback)
    
   
    rospy.spin()
