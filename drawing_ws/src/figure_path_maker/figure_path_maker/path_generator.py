import rclpy
from rclpy.node import Node
from figure_msgs.msg import PoseStampedArray
from figure_msgs.srv import PathGenerator
from geometry_msgs.msg import PoseStamped
#from figure_parameters import FigureParameters
from math import cos, sin, pi
import numpy as np

class PathGen(Node):
  def __init__(self):
    super().__init__('pathGen')
    self.srv = self.create_service(PathGenerator, 'PathGenerator', self.path_generation_callback)
    self.get_logger().info('Initialized path generator node')
  
  def path_generation_callback(self, request, response):
    # Obtaining the figure parameters
    self.radius = request.figure.radius # radius of the figure
    self.length = request.figure.length # length of the figure
    self.width = request.figure.width # width of the figure
    self.n = 8 # number of points per face
    self.xO = 0.0 # x center of the figure
    self.yO = 0.0 # y center of the figure
    self.zO = 0.0 # z center of the figure
    # Generating the path
    self.figure_poses = PoseStampedArray()
    builtFlag = self.buildPoses(request.figure.figure_id)

    # Hardcode
    #builtFlag = True
    #posestamp = PoseStamped()
    #posestamp.pose.position.x = float(19)
    #posestamp.pose.position.y = float(80)
    #posestamp.pose.position.z = float(5)
    #self.figure_poses.poses.append(posestamp)
    

    # 
    if builtFlag:
      response.poses.poses = self.figure_poses.poses
      self.get_logger().info('Path generated')
      return response
    else:
      self.get_logger().info('Path not generated')
      return response
  
  # Building the poses for the different figures
  def buildPoses(self, value):
    builtFlag = False
    if(value == 1):
      builtFlag = self.buildSphere()
    elif(value == 2):
      builtFlag = self.buildcube()
    elif(value == 3):
      builtFlag = self.buildcuboid()
    elif(value == 4):
      builtFlag = self.buildcylinder()
    elif(value == 5):
      builtFlag = self.buildhexagonal()
    elif(value == 6):
      builtFlag = self.buildtriangular()
    elif(value == 7):
      builtFlag = self.buildcone()
    elif(value == 8):
      builtFlag = self.buildpyramid()
    elif(value == 9):
      builtFlag = self.buildtetaedro()
    else:
      self.get_logger().info('Invalid figure')
    return builtFlag

  # Building the poses for the sphere
  def buildSphere(self):
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    delta = 2*pi/self.n
    # Check for radius
    if(self.radius <= 0):
      self.get_logger().info('Invalid radius')
      return False
    # Circle in the xy plane
    z = self.zO
    for i in np.arange(0, int(2*pi), delta):
      x = self.xO + self.radius * cos(i)
      y = self.yO + self.radius * sin(i)
      self.appendPoststamp(x, y, z)
    x = self.xO + self.radius
    y = self.yO
    self.appendPoststamp(x, y, z)
    # Circle in the xz plane
    y = 0
    for i in np.arange(0, 2*pi, delta):
      x = self.xO + self.radius * cos(i)
      z = self.zO + self.radius * cos(i)
      self.appendPoststamp(x, y, z)
    x = self.xO + self.radius
    z = self.zO
    self.appendPoststamp(x, y, z)
    # Circle in the yz plane
    x = 0
    for i in np.arange(0, 2*pi, delta):
      y = self.yO + self.radius * sin(i)
      z = self.zO + self.radius * cos(i)
      self.appendPoststamp(x, y, z)
    y = self.yO + self.radius
    z = self.zO
    self.appendPoststamp(x, y, z)
    return True

  # Building the poses for the cube
  def buildcube(self):
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    delta = 4*self.length/self.n
    # Check for length and width
    if(self.length == 0):
      self.length = self.width
      self.get_logger().info('No length provided, using width as length')
    if(self.length <= 0):
      self.get_logger().info('Invalid length')
      return False
    # Front face
    x = self.xO + self.length/2
    y = self.yO - self.length/2
    z = self.zO + self.length/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      y += i
      self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      z -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      y -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      z += i
      self.appendPoststamp(x, y, z)
    # Right face
    x = self.xO + self.length/2
    y = self.yO + self.length/2
    z = self.zO + self.length/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      x -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      z -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      x += i
      self.appendPoststamp(x, y, z)
    # Back face
    x = self.xO - self.length/2
    y = self.yO + self.length/2
    z = self.zO + self.length/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      y -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      z -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      y += i
      self.appendPoststamp(x, y, z)
    # Left face
    x = self.xO - self.length/2
    y = self.yO - self.length/2
    z = self.zO + self.length/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      x += i
      self.appendPoststamp(x, y, z)
    z -= self.length
    self.appendPoststamp(x, y, z)
    for i in np.arange(delta, self.length + delta, delta):
      x -= i
      self.appendPoststamp(x, y, z)
    return True

  # Building the given pose
  def appendPoststamp(self, x_p, y_p, z_p):
    posestamp = PoseStamped()
    posestamp.pose.position.x = float(x_p)
    posestamp.pose.position.y = float(y_p)
    posestamp.pose.position.z = float(z_p)
    self.figure_poses.poses.append(posestamp)
    return

def main(args=None):
  rclpy.init(args=args)
  path_gen_node = PathGen()
  rclpy.spin(path_gen_node)
  path_gen_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()