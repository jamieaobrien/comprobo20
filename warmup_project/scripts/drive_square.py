#!/usr/bin/env python

import rospy
import math
import threading
import sys
import termios
import numpy as np
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

class DriveSquare(object):
    def __init__(self):
        # sys
        self.settings_ = termios.tcgetattr(sys.stdin)

        # initialize node
        rospy.init_node("drive_square")
        
        self.pose = Pose2D()

        # Initialize subscriber and publisher
        self.odom_sub = rospy.Subscriber("odom",Odometry,self.callback)
        #self.tf_sub = rospy.Subscriber("tf",tf2,self.tf_cb)
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

        # Spin to keep python from exiting until the node is stopped
        #rospy.spin()

    def callback(self,data):
        self.pose.x = data.pose.pose.position.x
        self.pose.y = data.pose.pose.position.y
        self.pose.theta = 2.0 * np.arctan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
               

    def run(self):
        try:
            while not rospy.is_shutdown():
                cmd_vel = Twist()
                #if math.sqrt((self.pose.x-self.pose0.x)**2 + (self.pose.y-self.pose0.y)**2) < 1.0:
                if math.sqrt((self.pose.x-self.x0)**2 + (self.pose.y-self.y0)**2) < 0.99:
                    #print(math.sqrt((self.pose.x-self.x0)**2 + (self.pose.y-self.y0)**2))
                    #print(self.pose,self.pose0)
                    cmd_vel.linear.x = self.lin_speed * self.x_scale * (1- math.sqrt((self.pose.x-self.x0)**2 + (self.pose.y-self.y0)**2))
                    cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)  
                    #print("go forward")
                elif abs((self.th0) - (self.pose.theta)) % 1.57 < 1.569 or (self.pose.theta > 6.0 and self.th0 < -6.0):
                    print(self.pose.theta,self.th0,abs((self.th0) - (self.pose.theta)) % 1.57)
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = self.ang_speed * self.z_scale * abs(1.57-abs(self.pose.theta-self.th0)%1.57)
                    self.cmd_vel_pub.publish(cmd_vel)  
                    print("turn")   
                else:
                    self.x0 = self.pose.x
                    self.y0 = self.pose.y
                    self.th0 = self.pose.theta 
                    if self.th0 > 6.0:
                        self.th0 = -1*self.th0
                    cmd_vel.linear.x = cmd_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd_vel)
        except Exception as e:
            rospy.loginfo('{}'.format(e))
        finally:
            cmd_vel.linear.x = cmd_vel.linear.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)


if __name__=="__main__":
    drive_square = DriveSquare()
    drive_square.run()
