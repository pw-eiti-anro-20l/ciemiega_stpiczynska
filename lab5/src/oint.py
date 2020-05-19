#!/usr/bin/env python

import rospy
from lab5.srv import oint_control
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math


freq = 50
prev_pos = (1.2, 0., 0.)
prev_q = (0., 0., 0., 1.)
path = Path()


def handle_interpolation(req):


    #if req.t <= 0 or not -3.14 <= req.x <= 3.14 or not -3.14 <= req.y <= 0 or not -1 <= req.z <= 1:
     #   return False

    global prev_pos
    global prev_q
    new_pos = (req.x, req.y, req.z)
    new_q = (req.qx, req.qy, req.qz, req.qw)
    rate = rospy.Rate(freq)
    current_pos = rospy.wait_for_message('joint_states', JointState, timeout = 10).position

    current_time = 0.
    frames_number = int(math.ceil(req.t * freq))

    for i in range(frames_number+1):
        x = compute_int(prev_pos[0], new_pos[0], req.t, current_time, req.i)
        y = compute_int(prev_pos[1], new_pos[1], req.t, current_time, req.i)
        z = compute_int(prev_pos[2], new_pos[2], req.t, current_time, req.i)
        qx = compute_int(prev_q[0], new_q[0], req.t, current_time, req.i)
        qy = compute_int(prev_q[1], new_q[1], req.t, current_time, req.i)
        qz = compute_int(prev_q[2], new_q[2], req.t, current_time, req.i)
        qw = compute_int(prev_q[3], new_q[3], req.t, current_time, req.i)

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        pub.publish(pose)

        path.header = pose.header
        path.poses.append(pose)
        path_pub.publish(path)
        
        current_time = current_time + 1.0 / freq
        rate.sleep()

    prev_pos = new_pos
    prev_q = new_q
    return True


def compute_int(start_j, last_j, time, current_time, i):
    if i == 'tri':
        return compute_tri(start_j, last_j, time, current_time)
    else:
        return compute_const(start_j, last_j, time, current_time)


def compute_const(start_j, last_j, time, current_time):
    return start_j + (float(last_j - start_j) / time) * current_time


def compute_tri(start_j, last_j, time, current_time):
    h = 2. * float(last_j - start_j) / time
    ratio = h / (time / 2.)
    if current_time < time / 2.:
        return start_j + current_time**2 * ratio / 2.
    else:
        return last_j - (time-current_time)**2 * ratio / 2.

if __name__ == "__main__":




    rospy.init_node('oint_srv')
    pub = rospy.Publisher('oint_pose', PoseStamped, queue_size=10)
 #   pub = rospy.Publisher('jint_control', JointState, queue_size=10)
    path_pub = rospy.Publisher('pathing', Path, queue_size=10)
    s = rospy.Service('oint', oint_control, handle_interpolation)
    rospy.spin()
