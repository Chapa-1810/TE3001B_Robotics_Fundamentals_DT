from figure_msgs.msg import PoseStampedArray
from geometry_msgs.msg import PoseStamped
from math import cos, sin, pi, floor, sqrt
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
    # Check for errors
    if(self.radius <= 0):
      return "Invalid radius"
    if(self.n <= 0):
      return "Invalid detail"
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    delta = 2*pi/(7 + self.n)
    # Circle in the xy plane
    for i in np.arange(0, 2*pi, delta):
      x = self.xO + self.radius * cos(i)
      y = self.yO + self.radius * sin(i)
      self.appendPoststamp(x, y, z)
    x = self.xO + self.radius
    y = self.yO
    self.appendPoststamp(x, y, z)
    # Circle in the xz plane
    y = self.yO
    for i in np.arange(0, 2*pi, delta):
      x = self.xO + self.radius * cos(i)
      z = self.zO + self.radius * cos(i)
      self.appendPoststamp(x, y, z)
    x = self.xO + self.radius
    z = self.zO
    self.appendPoststamp(x, y, z)
    # Circle in the yz plane
    x = self.xO
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
    # Check for errors
    if(self.length == 0):
      self.length = self.width
    if(self.length <= 0):
      return "Invalid length or width"
    if(self.n <= 0):
      return "Invalid detail"
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    delta = self.length/self.n
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
  
  # Building the poses for the cuboid
  def buildcuboid(self):
    # Check for errors
    if(self.length <= 0):
      return "Invalid length"
    if(self.width <= 0):
      return "Invalid width"
    if(self.n <= 0):
      return "Invalid detail"
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    deltaL = self.length/self.n
    deltaW = self.width/(self.n * floor(self.width/self.length))
    # Front face
    x = self.xO + self.width/2
    y = self.yO - self.length/2
    z = self.zO + self.length/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      y += i
      self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      z -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      y -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      z += i
      self.appendPoststamp(x, y, z)
    # Right face
    x = self.xO + self.width/2
    y = self.yO + self.length/2
    z = self.zO + self.length/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(deltaW, self.width + deltaW, deltaW):
      x -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      z -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(deltaW, self.width + deltaW, deltaW):
      x += i
      self.appendPoststamp(x, y, z)
    # Back face
    x = self.xO - self.width/2
    y = self.yO + self.length/2
    z = self.zO + self.length/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      y -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      z -= i
      self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      y += i
      self.appendPoststamp(x, y, z)
    # Left face
    x = self.xO - self.width/2
    y = self.yO - self.length/2
    z = self.zO + self.length/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(deltaW, self.width + deltaW, deltaW):
      x += i
      self.appendPoststamp(x, y, z)
    z -= self.length
    self.appendPoststamp(x, y, z)
    for i in np.arange(deltaW, self.width + deltaW, deltaW):
      x -= i
      self.appendPoststamp(x, y, z)
    return ""

  # Building the poses for the cylinder
  def buildcylinder(self):
    # Check for errors
    if(self.width == 0):
      self.width = self.length
    if(self.radius <= 0):
      return "Invalid radius"
    if(self.width <= 0):
      return "Invalid width or length"
    if(self.n <= 0):
      return "Invalid detail"
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    sign = 1
    deltaR = 2*pi/(7 + self.n)
    deltaW = self.width/(self.n * floor(self.width/(2*self.radius)))
    # Circular faces
    x = self.xO + self.width/2
    for i in np.arange(0, 2*pi, deltaR):
      y = self.yO + self.radius * cos(i)
      z = self.zO + self.radius * sin(i)
      self.appendPoststamp(x, y, z)
      sign *= -1
      # Traverse the width
      for j in np.arange(deltaW, self.width + deltaW, deltaW):
        x += sign * j
        self.appendPoststamp(x, y, z)
  
  # Building the poses for the hexagonal
  def buildhexagonal(self):
    # Check for errors
    if(self.length == 0):
      self.length = self.radius
    if(self.length <= 0):
      return "Invalid length or radius"
    if(self.width <= 0):
      return "Invalid width"
    if(self.n <= 0):
      return "Invalid detail"
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    sign = 1
    angle = 60
    deltaL = self.length/self.n
    deltaW = self.width/(self.n * floor(self.width/self.length))
    # Hexagonal face
    x = self.xO + self.width/2
    y = self.yO + self.length/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(0, 6, 1):
      angle += 60
      # Traverse the length
      for j in np.arange(deltaL, self.length + deltaL, deltaL):
        y += self.length * cos(angle)
        z += self.length * sin(angle)
        self.appendPoststamp(x, y, z)
      sign *= -1
      # Traverse the width
      for k in np.arange(deltaW, self.width + deltaW, deltaW):
        x += sign * k
        self.appendPoststamp(x, y, z)
    return ""
  
  # Building the poses for the triangular
  def buildtriangular(self):
    # Check for errors
    if(self.length == 0):
      self.length = self.radius * sqrt(3)
    if(self.length <= 0):
      return "Invalid length or radius"
    if(self.width <= 0):
      return "Invalid width"
    if(self.n <= 0):
      return "Invalid detail"
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    sign = 1
    angle = 0
    deltaL = self.length/self.n
    deltaW = self.width/(self.n * floor(self.width/self.length))
    # Triangular face
    x = self.xO + self.width/2
    y = self.yO + self.length / sqrt(3) * cos(11/6*pi)
    z = self.zO + self.length / sqrt(3) * sin(11/6*pi)
    self.appendPoststamp(x, y, z)
    for i in np.arange(0, 3, 1):
      angle += 120
      # Traverse the length
      for j in np.arange(deltaL, self.length + deltaL, deltaL):
        y += self.length * cos(angle)
        z += self.length * sin(angle)
        self.appendPoststamp(x, y, z)
      sign *= -1
      # Traverse the width
      for k in np.arange(deltaW, self.width + deltaW, deltaW):
        x += sign * k
        self.appendPoststamp(x, y, z)
    return ""

  # Building the poses for the cone
  def buildcone(self):
    # Check for errors
    if(self.width == 0):
      self.width = self.length
    if(self.radius <= 0):
      return "Invalid radius"
    if(self.width <= 0):
      return "Invalid width or length"
    if(self.n <= 0):
      return "Invalid detail"
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    deltaR = 2*pi/(7 + self.n)
    #deltaW = self.width/(self.n * floor(self.width/(2*self.radius)))
    # Circular face
    x = self.xO + self.length/2
    for i in np.arange(0, 2*pi, deltaR):
      x = self.yO + self.radius * cos(i)
      y = self.zO + self.radius * sin(i)
      self.appendPoststamp(x, y, z)
      # Traverse the length
      xT = x
      yT = y
      x = self.xO
      y = self.yO
      z += self.width
      self.appendPoststamp(x, y, z)
      x = xT
      y = yT
      z -= self.width
      self.appendPoststamp(x, y, z)
    return "" 
    
  # Building the poses for the pyramid
  def buildpyramid(self):
    # Check for errors
    if(self.length <= 0):
      return "Invalid length"
    if(self.width <= 0):
      return "Invalid width"
    if(self.n <= 0):
      return "Invalid detail"
    # Initialize variables
    x = self.xO
    y = self.yO
    z = self.zO
    deltaL = self.length/self.n
    #deltaW = self.width/(self.n * floor(self.width/self.length))
    # Quadrangular face
    x = self.xO + self.length/2
    y = self.yO - self.length/2
    z = self.zO - self.width/2
    self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      y += i
      self.appendPoststamp(x, y, z)
    xT = x
    yT = y
    x = self.xO
    y = self.yO
    z += self.width
    self.appendPoststamp(x, y, z)
    x = xT
    y = yT
    z -= self.width
    self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      x -= i
      self.appendPoststamp(x, y, z)
    xT = x
    yT = y
    x = self.xO
    y = self.yO
    z += self.width
    self.appendPoststamp(x, y, z)
    x = xT
    y = yT
    z -= self.width
    self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      y -= i
      self.appendPoststamp(x, y, z)
    xT = x
    yT = y
    x = self.xO
    y = self.yO
    z += self.width
    self.appendPoststamp(x, y, z)
    x = xT
    y = yT
    z -= self.width
    self.appendPoststamp(x, y, z)
    for i in np.arange(deltaL, self.length + deltaL, deltaL):
      x += i
      self.appendPoststamp(x, y, z)
    xT = x
    yT = y
    x = self.xO
    y = self.yO
    z += self.width
    self.appendPoststamp(x, y, z)
    x = xT
    y = yT
    z -= self.width
    self.appendPoststamp(x, y, z)
    return ""

  # Building the poses for the tetrahedron
  


  # Building the given pose
  def appendPoststamp(self, x_p, y_p, z_p):
    posestamp = PoseStamped()
    posestamp.pose.position.x = float(x_p)
    posestamp.pose.position.y = float(y_p)
    posestamp.pose.position.z = float(z_p)
    self.poses.poses.append(posestamp)
    return