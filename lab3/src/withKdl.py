#! /usr/bin/python

import json
import rospy
import PyKDL
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
    kdlChain =PyKDL.Chain()   
    Frame = PyKDL.Frame();

    d=0
    theta=0
    i=1
    for instance in dhJson:
        oneInstance= json.loads(json.dumps(instance))
        lastD= d
        lastTheta=theta
        a = oneInstance["a"]
        d = oneInstance["d"]
        alpha=oneInstance["alpha"]
        theta = oneInstance["theta"]
        if i!= 1:
            kdlChain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.TransZ), Frame.DH(a, alpha, lastD, lastTheta)))
        i=i+1
    kdlChain.addSegment(PyKDL.Segment(PyKDL.Joint(PyKDL.Joint.TransZ), Frame.DH(0, 0, d, theta)))
      
      
    jointDisplacement = PyKDL.JntArray(kdlChain.getNrOfJoints())

    jointDisplacement[0] = data.position[0]
    jointDisplacement[1] = data.position[1]
    jointDisplacement[2] = data.position[2]

    fk = PyKDL.ChainFkSolverPos_recursive(kdlChain)

    finalFrame = PyKDL.Frame()
    fk.JntToCart(jointDisplacement, finalFrame)
    quaternion = finalFrame.M.GetQuaternion()
    
    pose = PoseStamped()
    pose.header.frame_id = 'base_link'
    pose.header.stamp = rospy.Time.now()

    pose.pose.position.x = finalFrame.p[0]
    pose.pose.position.y = finalFrame.p[1]
    pose.pose.position.z = finalFrame.p[2]
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    pub.publish(pose)
    


if __name__ == '__main__':
    
    rospy.init_node('KDL_DKIN', anonymous=False)
    dhJson ={}
    restrictionsJson ={}
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../restrictions.json', 'r') as file:
        restrictionsJson= json.loads(file.read())
   
    with open(os.path.dirname(os.path.realpath(__file__)) + '/../dh_data.json', 'r') as file:
        dhJson= json.loads(file.read())

    pub = rospy.Publisher('KdlAxes', PoseStamped, queue_size=10)
    rospy.Subscriber("joint_states", JointState , callback)
    
   
    rospy.spin()
