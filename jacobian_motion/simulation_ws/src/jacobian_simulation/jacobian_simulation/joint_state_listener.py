#!/usr/bin/python3

# create a simple joint state listener

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('joint_state_listener')
        self.joint_states_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.joint_states_subscriber  # prevent unused variable warning
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))
        self.joint_states = None
    
    def joint_states_callback(self, msg):
        self.joint_states = msg
        self.get_logger().info('Joint states: {0}'.format(self.joint_states))
    
if __name__ == '__main__':
    joint_state_listener = JointStateListener()
    rclpy.spin(joint_state_listener)
    joint_state_listener.destroy_node()
    rclpy.shutdown()
    