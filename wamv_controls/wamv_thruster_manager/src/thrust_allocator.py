#!/usr/bin/env python

import numpy
import rospy
import tf
import tf.transformations as trans
import tf2_ros
from std_msgs.msg import Float32
from os.path import isdir, join
import yaml
from time import sleep
from geometry_msgs.msg import Wrench
import xml.etree.ElementTree as etree

class ThrustAllocator:
    """
    The thrust allocator computes thrust forces for each thruster
    and publish them directly for the gazebo plugin
    """

    def __init__(self):
        """Class constructor."""
        # This flag will be set to true once the thruster allocation matrix is
        # available
        self.ready = False

        # Thruster allocation matrix: transform thruster inputs to force/torque
        self.configuration_matrix = None
        if rospy.has_param('~tam'):
            tam = rospy.get_param('~tam')
            self.configuration_matrix = numpy.array(tam)
            # Set number of thrusters from the number of columns
            self.n_thrusters = self.configuration_matrix.shape[1]
            rospy.loginfo('Thruster allocation matrix provided!')
            rospy.loginfo('TAM=')
            rospy.loginfo(self.configuration_matrix)
            self.thrust = numpy.zeros(self.n_thrusters)

        # if not self.update_tam():
        #    raise rospy.ROSException('No thrusters found')

        # (pseudo) inverse: force/torque to thruster inputs
        self.inverse_configuration_matrix = None
        if self.configuration_matrix is not None:
            self.inverse_configuration_matrix = numpy.linalg.pinv(
                self.configuration_matrix)
        rospy.loginfo('Inverse TAM= %s', str(self.inverse_configuration_matrix))

        self.left_front_pub = rospy.Publisher("left_front_thrust_cmd", Float32, queue_size=10)
        self.left_rear_pub = rospy.Publisher("left_rear_thrust_cmd", Float32, queue_size=10)
        self.right_front_pub = rospy.Publisher("right_front_thrust_cmd", Float32, queue_size=10)
        self.right_rear_pub = rospy.Publisher("right_rear_thrust_cmd", Float32, queue_size=10)

        # Subscriber to the wrench to be applied on the UUV
        self.input_sub = rospy.Subscriber('/wamv/thruster_manager/input',
                                          Wrench, self.input_callback)

        self.max_thrust = 2000
        self.scale_factor = 800

        self.left_front_msg = Float32()
        self.right_front_msg = Float32()
        self.left_rear_msg = Float32()
        self.right_rear_msg = Float32()

        self.ready = True
        rospy.loginfo('ThrustAllocator: ready')

    def command_thrusters(self):
        """Publish the thruster input into their specific topic."""
        if self.thrust is None:
            return

        self.right_front_msg.data = self.thrust[1]/self.scale_factor
        self.right_front_pub.publish(self.right_front_msg)

        self.left_front_msg.data = self.thrust[0]/self.scale_factor
        self.left_front_pub.publish(self.left_front_msg)

        self.right_rear_msg.data = self.thrust[3]/self.scale_factor
        self.right_rear_pub.publish(self.right_rear_msg)

        self.left_rear_msg.data = self.thrust[2]/self.scale_factor
        self.left_rear_pub.publish(self.left_rear_msg)


    def input_callback(self, msg):
        """
        Callback to the subscriber that receiver the wrench to be applied on
        WAMV's BODY frame.
        @param msg Wrench message
        """
        if not self.ready:
            return

        force = numpy.array((msg.force.x, msg.force.y, msg.force.z))
        torque = numpy.array((msg.torque.x, msg.torque.y, msg.torque.z))

        # This mode assumes that the wrench is given wrt thruster manager
        # configured base_link reference
        self.publish_thrust_forces(force, torque)

        self.last_update = rospy.Time.now()

    def publish_thrust_forces(self, control_forces, control_torques,
                              frame_id=None):
        if not self.ready:
            return

        gen_forces = numpy.hstack(
            (control_forces, control_torques)).transpose()
        self.thrust = self.compute_thruster_forces(gen_forces)
        self.command_thrusters()

    def compute_thruster_forces(self, gen_forces):
        """Compute desired thruster forces using the inverse configuration
        matrix.
        """
        # Calculate individual thrust forces
        thrust = self.inverse_configuration_matrix.dot(gen_forces)
        # Obey limit on max thrust by applying a constant scaling factor to all
        # thrust forces
        for i in range(self.n_thrusters):
            if abs(thrust[i]) > self.max_thrust:
                thrust[i] = numpy.sign(thrust[i]) * self.max_thrust
        return thrust

if __name__ == '__main__':
    rospy.init_node('thrust_allocator')

    try:
        node = ThrustAllocator()
        rospy.spin()
    except rospy.ROSInterruptException:
        print 'ThrustAllocatorNode::Exception'
    print 'Leaving ThrustAllocatorNode'
