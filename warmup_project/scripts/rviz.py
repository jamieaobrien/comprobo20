#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

class markerNode():
    def __init__(self):
        pub = rospy.Publisher('visualization_messages/Marker', Marker, queue_size=10)

        rospy.init_node('markerNode')

        rate = rospy.Rate(10)

        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.id = 0
        self.marker.type = 2 # sphere
        self.marker.action = 0
        self.marker.pose.position.x = 1
        self.marker.pose.position.y = 2
        self.marker.pose.position.z = 0.5
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0

        self.marker.color.r = 0.6
        self.marker.color.g = 0.27
        self.marker.color.b = 0.43
        self.marker.color.a = 1.0
        
        self.marker.lifetime = rospy.Duration(0)

        while not rospy.is_shutdown():
            pub.publish(self.marker)
            rate.sleep()


marker_node = markerNode()
