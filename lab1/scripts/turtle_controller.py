#!/usr/bin/env python

import rospy
import curses
from geometry_msgs.msg import Twist

def assign(key, controlKeys):
    newMsg = Twist()

    if key == ord(controlKeys['forward']):
        newMsg.linear.x = 2
    elif key == ord(controlKeys['backward']):
        newMsg.linear.x = -2
    elif key == ord(controlKeys['left']):
        newMsg.angular.z = 2
    elif key == ord(controlKeys['right']):
        newMsg.angular.z = -2
    
    return newMsg


def controller():
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(50) # 50hz

    stdcr = curses.initscr()
    stdcr.nodelay(1)

    controlKeys = rospy.get_param('controller')

    newMsg = Twist()

    while not rospy.is_shutdown():
        key = stdcr.getch()
        newMsg = assign(key, controlKeys)
        #rospy.loginfo(newMsg)
        pub.publish(newMsg)
        rate.sleep()

if __name__ == '__main__':
    try:

        controller()
    except rospy.ROSInterruptException:
        pass
