#!/usr/bin/env python -W ignore::DeprecationWarning
"""
Starter script for EE106B grasp planning lab
Author: Chris Correa
"""
import numpy as np
import sys
import argparse

import rospy
import tf

from geometry_msgs.msg import Twist

class CircleTurn:
    def __init__(self):
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        self.rate = rospy.Rate(10)

    def publish(self, linear_velocity, steering_rate, timeout=False):
        out = Twist()
        out.linear.x = linear_velocity
        out.angular.z = steering_rate

        if timeout is False:
            self.pub.publish(out)
            self.rate.sleep()
        else:
            for i in range(10 * timeout):
                self.pub.publish(out)
                self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('circle_turn')
    circleTurn = CircleTurn()
    circleTurn.publish(0, 2, timeout=6)