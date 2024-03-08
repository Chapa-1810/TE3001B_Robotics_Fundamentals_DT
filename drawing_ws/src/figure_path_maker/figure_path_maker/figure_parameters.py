from figure_msgs.msg import PoseStampedArray
from geometry_msgs.msg import PoseStamped
from math import cos, sin, pi
import numpy as np

class FigureParameters:
  def __init__(self):
    self.poses = PoseStampedArray()
    self.radius = 0.0
    self.length = 0.0
    self.width = 0.0
    self.n = 0
    self.xO = 0.0
    self.yO = 0.0
    self.zO = 0.0
    
  # Building the poses for the different figures
  def buildPoses(self, value):
    builtFlag = ""
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
      builtFlag = "Invalid figure"
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
      return "Invalid radius"
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
    return ""
  
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
    if(self.length <= 0):
      return "Invalid length or width"
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
    return ""
  
  # Building the given pose
  def appendPoststamp(self, x_p, y_p, z_p):
    posestamp = PoseStamped()
    posestamp.pose.position.x = float(x_p)
    posestamp.pose.position.y = float(y_p)
    posestamp.pose.position.z = float(z_p)
    self.poses.poses.append(posestamp)
    return