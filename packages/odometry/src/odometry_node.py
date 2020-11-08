#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, Bool

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)

        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self.length_per_tick = 2*np.pi*self._radius/135
        self.counter_l = 0
        self.counter_r = 0
        self.counter_l_past = 0
        self.counter_r_past = 0
        self.difference_l = 0
        self.difference_r = 0
        self.distance_l = 0.
        self.distance_r = 0.
        self.save = 0
        self.save_difference = 0.57
        self.save_tick_start_l = 0
        self.save_tick_end_l = 0
        self.save_tick_start_r = 0
        self.save_tick_end_r = 0
        self.save_radius_l = 0.
        self.save_radius_r = 0.
        self.save_radius_end = 0.


        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber("~left_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_encoder_data_l, queue_size=1)
        self.sub_encoder_ticks_right = rospy.Subscriber("~right_wheel_encoder_node/tick", WheelEncoderStamped, self.cb_encoder_data_r, queue_size=1)
        #self.sub_executed_commands = rospy.Subscriber("wheels_driver_node", WheelsCmdStamped, self.cb_executed_commands, queue_size=1)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher("~left_encoder_distance", Float32, queue_size=1)
        self.pub_integrated_distance_right = rospy.Publisher("~right_encoder_distance", Float32, queue_size=1)

        self.log("Initialized")

        self.sub_save_radius = rospy.Subscriber("~save_radius", Bool, self.cb_save_radius, queue_size=1)

    def cb_encoder_data_l(self,  msg):
        """ Update encoder distance information from ticks.
        """
        self.counter_l_past = self.counter_l
        self.counter_l = msg.data

        self.difference_l = self.counter_l - self.counter_l_past
        self.distance_l += self.difference_l * self.length_per_tick
        self.pub_integrated_distance_left.publish(self.distance_l)




    def cb_encoder_data_r(self, msg):
        """ Update encoder distance information from ticks.
        """
        self.counter_r_past = self.counter_r
        self.counter_r = msg.data

        self.difference_r = self.counter_r - self.counter_r_past
        self.distance_r += self.difference_r * self.length_per_tick
        self.pub_integrated_distance_right.publish(self.distance_r)

    def cb_save_radius(self, msg):
        rospy.loginfo(self._radius)
        self.save = msg.data
        if self.save:
            self.save_tick_start_l = self.counter_l
            self.save_tick_start_r = self.counter_r

        else:
            self.save_tick_end_l = self.counter_l
            self.save_tick_end_r = self.counter_r

            self.save_radius_l = self.save_difference/(2*np.pi)*135/(self.save_tick_end_l - self.save_tick_start_l)
            self.save_radius_r = self.save_difference / (2*np.pi) * 135 / (self.save_tick_end_r - self.save_tick_start_r )
            self.save_radius_end = 0.5*(self.save_radius_l + self.save_radius_r)
            rospy.set_param(f'/{self.veh_name}/kinematics_node/radius', self.save_radius_end)
            rospy.loginfo(self.save_radius_l)
            rospy.loginfo(self.save_radius_r)



    # def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """

if __name__ == '__main__':
    node = OdometryNode(node_name='odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")