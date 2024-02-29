import numpy as np
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JacobianGoal(Node):
  def __init__(self):
    rclpy.init()
    super().__init__('jacobian_goal')

    self.joint_states_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
    self.joint_states_subscriber  # prevent unused variable warning
    self.joint_states_publisher = self.create_publisher(JointState, 'joint_states', 10)
    self.nodeName = self.get_name()
    self.get_logger().info("{0} started".format(self.nodeName))
    self.joint_states = None

  def jacobian(self, dennavit_harten):
    jacobian = np.zeros((6, 3))
    homogen = np.zeros((4, 4))
    rots = []
    ds = []

    for i in range(3):
      homogen = np.array([[np.cos(dennavit_harten[i, 0]), -np.sin(dennavit_harten[i, 0])*np.cos(dennavit_harten[i, 1]), np.sin(dennavit_harten[i, 0])*np.sin(dennavit_harten[i, 1]), dennavit_harten[i, 2]*np.cos(dennavit_harten[i, 0])],
                          [np.sin(dennavit_harten[i, 0]), np.cos(dennavit_harten[i, 0])*np.cos(dennavit_harten[i, 1]), -np.cos(dennavit_harten[i, 0])*np.sin(dennavit_harten[i, 1]), dennavit_harten[i, 2]*np.sin(dennavit_harten[i, 0])],
                          [0, np.sin(dennavit_harten[i, 1]), np.cos(dennavit_harten[i, 1]), dennavit_harten[i, 3]],
                          [0, 0, 0, 1]])
      # Grab rotation matrix from homogen matrix
      rots.append(homogen[:3, :3])
      # Grab displacement from homogen matrix
      ds.append(homogen[:3, 3])

    # Calculate jacobian
    for i in range(3):
      jacobian[:3, i] = np.cross(rots[i][:, 2], ds[2] - ds[i])
      jacobian[3:, i] = rots[i][:, 2]

    return jacobian 

  def joint_states_callback(self, msg):
    self.joint_states = msg
  
  def start(self):
    vx = 0
    vy = 0
    vz = 0
    wx = 0
    wy = 0
    wz = 0
    
    a1 = 0.025
    a2 = 0.05
    a3 = 0.4
    
    alpha1 = 1.57
    alpha2 = 0
    alpha3 = 0
    theta1 = 0
    theta2 = 0
    theta3 = 0
    d1 = 0
    d2 = 0
    d3 = 0
    
    dennavit_harten = np.array([[theta1, alpha1, a1, d1],
                                [theta2, alpha2, a2, d2],
                                [theta3, alpha3, a3, d3]])
    
    velocities_matrix = np.array([vx, vy, vz, wx, wy, wz])
    start_time = time.time()
    
    prev_time = time.time()
    while time.time() - start_time < 100:
      dennavit_harten[0, 0] = theta1
      dennavit_harten[1, 0] = theta2
      dennavit_harten[2, 0] = theta3
      inverse_jacobian = np.linalg.pinv(self.jacobian(dennavit_harten))
      wq1, wq2, wq3 = np.dot(inverse_jacobian, velocities_matrix)
      x = np.sin(time.time() - start_time) 
      y = np.cos(time.time() - start_time) 
      #x = 0.3 + 2*np.cos(time.time() - start_time) + 5*np.cos((2/3)*time.time() - start_time)
      #y = 0.3 + 2*np.sin(time.time() - start_time) - 5*np.sin((2/3)*time.time() - start_time)
      z = time.time() - start_time
      velocities_matrix = np.array([x, y, z, wx, wy, wz])
      #print(wq1, wq2, wq3)
      # integrate q1, q2, q3
      theta1 += wq1 * (time.time() - prev_time)
      theta2 += wq2 * (time.time() - prev_time)
      theta3 += wq3 * (time.time() - prev_time)
      
      joint_state = JointState()
      joint_state.header.stamp = self.get_clock().now().to_msg()
      joint_state.name = ['joint1', 'joint2', 'joint3']
      joint_state.position = [theta1, theta2, theta3]
      self.joint_states_publisher.publish(joint_state)
      
      prev_time = time.time()
      print(theta1, theta2, theta3)
      print("------")

if __name__ == '__main__':
  jacobian_goal = JacobianGoal()
  jacobian_goal.start()
  rclpy.spin(jacobian_goal)
  jacobian_goal.destroy_node()
  rclpy.shutdown()
  