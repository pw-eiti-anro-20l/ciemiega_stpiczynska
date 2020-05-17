#!/usr/bin/env python

from math import cos, sin
import sys
import rospy
from lab5.srv import oint_control


def circle():
    freq = 30.0
    ax = 0.1
    ay = 0.05
    az = 0.02
    th = 0.0
    dth = 3.1415 / freq

    t = 1.0 / freq
    z0 = 0.05
    x0 = 0.4
    y0 = 0.0

    rospy.wait_for_service('oint')
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        x = x0 + ax * cos(th)
        y = y0 + ay * sin(th)
        z = z0 + az * sin(th)
        interpolation = rospy.ServiceProxy('oint', oint_control)
        resp1 = interpolation(x, y, z, 0.0, 0.0, 0.0, 1.0, t, '')

        th = th + dth
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('circle')
    circle()
