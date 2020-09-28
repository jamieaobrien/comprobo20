#!/usr/bin/env python

from geometry_msgs.msg import Twist, Vector3, Pose, Point
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import rospy
from math import sin, cos, pi
from tf.transformations import euler_from_quaternion

class WallFollow(object):
    def __init__(self):
        # init node
        rospy.init_node ('wall_follow_node')
        # init cmd_vel publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        # init subscribers
        self.scan_sub = rospy.Subscriber("/stable_scan", LaserScan, self.scan_cb)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        # init cmd_vel
        self.cmd_vel = Twist(linear=Vector3(x=0),angular=Vector3(z=0))
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0

        # set up rviz marker for wall

        self.position = Vector3(x=0,y=0,z=0) # z is yaw
        self.marker = Marker(scale = Vector3(x = 0.05, y = 0.05, z = 1.0))
        self.marker.color.a = 0.5
        self.marker.color.r = 1
        self.marker.header.frame_id = "odom"
        self.marker.type = Marker.POINTS
        self.marker_pub = rospy.Publisher('wall_marker',Marker,queue_size=10)


        # set params
        self.wall_distance = 1.0 # Stay about 1 meter from wall
        self.ang_speed = 0.5
        self.lin_speed = 0.1 # default linear speed
        self.wall = False # True if a wall is visible
        self.direction = 1
        self.window = 45 # side scan window
        self.scan = [] # holds LaserScan
        self.kp = 0.2 # Proportional controller constant
        self.error = 0 # controller error

    def odom_cb(self,data):

        yaw = euler_from_quaternion((data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w))
        self.position.x = data.pose.pose.position.x
        self.position.y = data.pose.pose.position.y
        self.position.z = yaw

    def scan_cb(self,data):

        self.wall = False
        self.scan = []

        angle_range = list(range(0,self.window)) + list(range(360 - self.window, 360))
        for angle in angle_range:
            distance = data.ranges[angle]
            if distance == 0.0:
                continue
            if distance < 2*self.wall_distance:
                self.wall = True
                if angle <= self.window:
                    self.scan.append([distance, angle*pi/180])
                else:
                    self.scan.append([distance, (angle-360)*pi/180])

        if self.wall:
            self.scan.sort(key=lambda item: item[0])
            self.error = self.wall_distance - self.scan[0][0]
            self.wall_marker()

    def wall_marker(self):

        self.marker.header.stamp = rospy.Time.now()
        x = self.scan[0][0]*cos(self.scan[0][1])
        y = self.scan[0][0]*sin(self.scan[0][1])
        point = Point(x,y,0)
        self.marker.points.append(point)

    def run(self):
        while not rospy.is_shutdown():
            if self.wall:
                if self.scan[0][1] < 0.0:
                    self.direction = 1
                else:
                    self.direction = -1

                self.cmd_vel.angular.z = self.direction*self.kp*self.error
            else:
                self.cmd_vel.linear.x = self.lin_speed
                self.cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(self.cmd_vel)
            self.marker_pub.publish(self.marker)
            self.rate.sleep()

if __name__=='__main__':
    wall_follow = WallFollow()
    wall_follow.run()
