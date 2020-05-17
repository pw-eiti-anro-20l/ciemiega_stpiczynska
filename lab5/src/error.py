#! /usr/bin/python

import rospy
import math
import tf
import geometry_msgs


if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    diff_pub = rospy.Publisher('diff', geometry_msgs.msg.Point, queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
    	desired_pos = rospy.wait_for_message('/oint_pose', geometry_msgs.msg.PoseStamped, timeout = 10).pose.position
        (trans, rot) = listener.lookupTransform('/base_link', '/link_3', rospy.Time(0))

        diff = geometry_msgs.msg.Point()
        diff.x = desired_pos.x - trans[0]
        diff.y = desired_pos.y - trans[1]
        diff.z = desired_pos.z - trans[2] + 0.2
        diff_pub.publish(diff)

        rate.sleep()
