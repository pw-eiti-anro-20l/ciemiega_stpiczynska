#!/usr/bin/env python

import sys
import rospy
from lab5.srv import oint_control


def rectangle():
    freq = 30.0
    ax = 0.01
    ay = 0.01
    az = 0.005
    th = 0.0
    dth = 3.1415 / freq

    t = 1.0 / freq
    z = -0.1
    x0 = x = 0.25
    y0 = y = 0.2
    xLast = 0.5
    yLast = 0.3

    s1 = True
    s2 = s3 = s4 = False

    rospy.wait_for_service('oint')
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        if x < xLast and round(y, 2) == y0:
            x = x + ax
        elif round(x, 2) == xLast and y < yLast:
            y = y + ay
            z = z + az
        elif x > x0 and round(y, 2) == yLast:
            x = x - ax
        elif round(x, 2) == x0 and y > y0:
            y = y - ay
            z = z - az

        interpolation = rospy.ServiceProxy('oint', oint_control)
        resp1 = interpolation(x, y, z, 0.0, 0.0, 0.0, 1.0, t, '')

        th = th + dth
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('rectangle')
    rectangle()
