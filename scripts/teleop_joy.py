#!/usr/bin/env python
# -*- coding: utf-8 -*
import roslib
import rospy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Teleop:
    def __init__(self):
        rospy.init_node('teleop_joy')
        self.x_speed = rospy.get_param('~x_speed',0.3)
        self.y_speed = rospy.get_param('~y_speed',0)
        self.w_speed = rospy.get_param('~w_speed',1)

        self.active = 0
        self.cmd = Twist()
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)

        rospy.Subscriber("joy", Joy, self.callback)
        rate = rospy.Rate(rospy.get_param('~hz', 20))

        while not rospy.is_shutdown():
            rate.sleep()
            if self.active == 1:
                self.cmd_pub.publish(self.cmd)

    def callback(self, data):
        """ Receive joystick data, formulate String message. """

        if data.buttons[6] == 1: 
            self.cmd.linear.x = self.x_speed * data.axes[3];
            self.cmd.angular.z = self.w_speed * data.axes[0];
            self.cmd_pub.publish(self.cmd)
            self.active = 1
        else:
            self.cmd = Twist()
            self.cmd.linear.x = 0;
            self.cmd.angular.z = 0;
            self.cmd_pub.publish(self.cmd)
            self.active = 0

if __name__ == "__main__": Teleop()

