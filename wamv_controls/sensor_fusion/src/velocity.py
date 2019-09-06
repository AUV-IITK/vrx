#!/usr/bin/env python

import sys
import rospy
import numpy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, Vector3
import tf.transformations as trans
import time

imu_data = Imu()
gps_vel = Vector3Stamped()

def imu_cb (msg):
    global imu_data
    imu_data = msg

def vel_cb (msg):
    global gps_vel
    gps_vel = msg

def vel_fix(vel):
    q = imu_data.orientation
    q = numpy.array([q.x, q.y, q.z, q.w])
    org_vel = numpy.array([vel.x, vel.y, vel.z])
    trans_vel = trans.quaternion_matrix(q).transpose()[0:3,0:3].dot(org_vel)
    corr_vel = Vector3(trans_vel[0], trans_vel[1], trans_vel[2])
    # print ("corrected vel: \n{}".format(corr_vel))
    return corr_vel

if __name__ == '__main__':

    rospy.init_node('wamv_velocity')

    imu_sub = rospy.Subscriber("/imu/data", Imu, imu_cb)
    vel_sub = rospy.Subscriber("/gps/fix_velocity", Vector3Stamped, vel_cb)
    vel_pub = rospy.Publisher("/gps/velocity", Vector3, queue_size=10, latch=True)

    correct_velocity = Vector3()

    while not rospy.is_shutdown():
        correct_velocity.z = gps_vel.vector.z
        correct_velocity.x = -gps_vel.vector.y
        correct_velocity.y = gps_vel.vector.x

        correct_velocity = vel_fix(correct_velocity)
        vel_pub.publish(correct_velocity)
        time.sleep(0.05)
