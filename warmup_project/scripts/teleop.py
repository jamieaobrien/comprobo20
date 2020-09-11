#!/usr/bin/env python
import rospy
import threading
import tty
import select
import sys
import termios
from geometry_msgs.msg import Twist



class Teleop(object):
    def __init__(self):
        # print key commands:
        msg = """
        To move around:
        u  i  o
        j  k  l 
        m  ,  .

        To adjust speed:
        q: increase linear speed by 10%
        z: decrease linear speed by 10%
        w: increase angular speed by 10%
        x: decrease angular speed by 10%
        """
        print(msg)

        # sys
        self.settings_ = termios.tcgetattr(sys.stdin)
        self.key = None

        # initialize node
        rospy.init_node("teleop")
        
        # ROS parameters
        self.x_scale = rospy.get_param('~v_scale',1.0)
        self.z_scale = rospy.get_param('~z_scale',1.0)
        
        # initialize publisher
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        
        # set initial values
        self.x_vel = 1.0
        self.z_vel = 1.0

        # key binding values will act as scalars for x_vel and z_vel
        self.key_bindings = {
            'i' : [1.0,0.0],
            'j' : [0.0,-1.0],
            'k' : [0.0,0.0],
            'l' : [0.0,1.0],
            ',' : [-1.0,0.0],
            'u' : [1.0,1.0],
            'o' : [1.0,-1.0],
            'm' : [-1.0,1.0],
            '.' : [-1.0,-1.0] 
        }
        self.vel_bindings = {
            'q' : [1.1, 1.0],
            'z' : [0.9, 1.0],
            'w' : [1.0, 1.1],
            'x' : [1.0, 0.9]
        }

    
    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin],[],[],0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)
        return key

    def run(self):
        # This function handles the keyboard commands.
        # If the key is in key_bindings, this function will update the 
        # twist object to scale the velocities according to the direction 
        # indicated by the key bindings.
        # If the key is in vel_bindings, this function will scale x_vel
        # or z_vel by 10%.
        try:
            while not rospy.is_shutdown():
                cmd_vel = Twist()
                while self.key != '\x03':
                    self.key = self.getKey()
                    if self.key in self.key_bindings:
                        v,w = self.key_bindings[self.key]
                        cmd_vel.linear.x = v * self.x_scale * self.x_vel
                        cmd_vel.angular.z = w * self.z_scale * self.z_vel
                        self.cmd_vel_pub.publish(cmd_vel)
                    elif self.key in self.vel_bindings:
                        self.x_vel = self.x_vel * self.vel_bindings[self.key][0]
                        self.z_vel = self.z_vel * self.vel_bindings[self.key][1]
                    else:
                        pass
        except Exception as e:
            rospy.loginfo('{}'.format(e))
        finally:
            cmd_vel.linear.x = cmd_vel.linear.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings_)



if __name__=="__main__":
    teleop = Teleop()
    teleop.run()




