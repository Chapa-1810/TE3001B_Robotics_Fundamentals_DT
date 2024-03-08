from figure_msgs.msg import PoseStampedArray
from math import cos, sin, pi
import numpy as np

class FigureParameters:
    def __init__(self):
        self.figure_poses = PoseStampedArray()
        self.radius = 0.0
        self.length = 0.0
        self.width = 0.0
        self.n = 0
        self.xO = 0.0
        self.yO = 0.0
        self.zO = 0.0