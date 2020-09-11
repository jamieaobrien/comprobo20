#!/usr/bin/env python

import rospy
import threading
from geometry_msgs.msg import Twist

class DriveSquare(object):
    def callback(data):
        print(data)

    def __init__(self):
        # initialize node
        rospy.init_node("drive_square")

        # Initialize subscriber and publisher
        self.encoder_sub = rospy.Subscriber("encoder",self.callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)

        # Spin to keep python from exiting until the node is stopped
        rospy.spin()

    def leftTurn(self):
        pass 
    