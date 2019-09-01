#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Float32
from vrx_gazebo.msg import FloatStamped

scale = 800

class Node():
    def __init__(self):
        self.left_front_pub = rospy.Publisher("left_front_thrust_cmd", Float32, queue_size=10)
        self.left_rear_pub = rospy.Publisher("left_rear_thrust_cmd", Float32, queue_size=10)
        self.right_front_pub = rospy.Publisher("right_front_thrust_cmd", Float32, queue_size=10)
        self.right_rear_pub = rospy.Publisher("right_rear_thrust_cmd", Float32, queue_size=10)
        
        self.left_front_msg = Float32()
        self.right_front_msg = Float32()
        self.left_rear_msg = Float32()
        self.right_rear_msg = Float32()

        self.left_front_sub = rospy.Subscriber("/anahita/thrusters/0/input", FloatStamped, self.callback_lf)
        self.left_rear_sub = rospy.Subscriber("/anahita/thrusters/2/input", FloatStamped, self.callback_lr)
        self.right_front_sub = rospy.Subscriber("/anahita/thrusters/1/input", FloatStamped, self.callback_rf)
        self.right_rear_sub = rospy.Subscriber("/anahita/thrusters/3/input", FloatStamped, self.callback_rr)

    def callback_rf (self, msg):
        self.right_front_msg.data = msg.data/scale
        self.right_front_pub.publish(self.right_front_msg)
    
    def callback_lf (self, msg):
        self.left_front_msg.data = msg.data/scale
        self.left_front_pub.publish(self.left_front_msg)

    def callback_rr (self, msg):
        self.right_rear_msg.data = msg.data/scale
        self.right_rear_pub.publish(self.right_rear_msg)

    def callback_lr (self, msg):
        self.left_rear_msg.data = msg.data/scale
        self.left_rear_pub.publish(self.left_rear_msg)

if __name__ == '__main__':

    rospy.init_node('controlx')
    node = Node()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
