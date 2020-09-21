#!/usr/bin/env python

import rospy
import math
import threading
import sys
import termios
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WallFollow(object):
    def __init__(self):
        # sys
        self.settings_ = termios.tcgetattr(sys.stdin)

        # initialize node
        rospy.init_node("drive_square")
        
        self.pose = Pose2D()

        # Initialize subscriber and publisher
        self.odom_sub = rospy.Subscriber("odom",Odometry,self.callback)
        self.scan_sub = rospy.Subscriber("scan",LaserScan,self.scan_cb)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        
        # ROS parameters
        self.x_scale = rospy.get_param('~v_scale',1.0)
        self.z_scale = rospy.get_param('~z_scale',1.0)

        # Class params
       
        self.x0 = self.pose.x
        self.y0 = self.pose.y 
        self.th0 = self.pose.theta
        self.lin_speed = 1.0
        self.ang_speed = 1.0
        self.turn_ang = 0.0

    def callback(self,data):
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        self.pose.theta = 2.0 * np.arctan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
               
    def scan_cb(self,data):
        self.ranges = np.array(data.ranges)
        angles = np.where(self.ranges < 2.0)[0]
        min_ang = angles[0]
        max_ang = angles[0]
        for i in range(1,len(angles)):
            if angles[i] - angles[i-1] < 30:
                max_ang = angles[i]
            else:
                walls.append([min_ang,max_ang])
                min_ang = angles[i+1]


        self.turn_ang = 0.0
        self.speed_scale = min(np.mean(self.ranges[160:200]),1)


    def run(self):
        cmd_vel = Twist()
        try:
            while not rospy.is_shutdown():
                if abs((self.th0) - (self.pose.theta)) % (self.turn_ang) < self.turn_ang - 0.001:
                    print(self.pose.theta,self.th0,abs((self.th0) - (self.pose.theta)) % self.turn_ang)
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = self.ang_speed * self.z_scale * (self.pose.theta-self.th0)
                    self.cmd_vel_pub.publish(cmd_vel)  
                    print("turn")
                else:
                    cmd_vel.linear.x = self.lin_speed * self.x_scale * self.speed_scale
                    self.cmd_vel_pub.publish(cmd_vel)
        except Exception as e:
            rospy.loginfo('{}'.format(e))
        finally:
            cmd_vel.linear.x = cmd_vel.linear.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)


if __name__=="__main__":
    wall_follow = WallFollow()
    wall_follow.run()
