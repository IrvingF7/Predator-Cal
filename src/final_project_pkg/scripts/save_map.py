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

from nav_msgs.msg import OccupancyGrid

class Map:
    def __init__(self, output_file="map.txt"):
        self.output_file = output_file

    def publish_map(self):
        self.pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        

    def save_map(self):
        self.sub = rospy.Subscriber('/map', OccupancyGrid, self.save_map_callback)
        
    def save_map_callback(self, msg):
        f = open(self.output_file, 'w')
        f.write()

if __name__ == '__main__':
    rospy.init_node('map_topic', anonymous=True)
    circleTurn = Map()
    circleTurn.save_map()