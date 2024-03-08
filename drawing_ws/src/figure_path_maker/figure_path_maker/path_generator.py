import rclpy
from rclpy.node import Node
from figure_msgs.srv import PathGenerator
from figure_path_maker.figure_parameters import FigureParameters

class PathGen(Node):
  def __init__(self):
    super().__init__('pathGen')
    self.srv = self.create_service(PathGenerator, 'PathGenerator', self.path_generation_callback)
    self.get_logger().info('Initialized path generator node')
  
  def path_generation_callback(self, request, response):
    # Obtaining the figure parameters
    figure = FigureParameters()
    figure.radius = request.figure.radius # radius of the figure
    figure.length = request.figure.length # length of the figure
    figure.width = request.figure.width # width of the figure
    figure.n = 8 # detail (number of points per face)
    figure.xO = 0.0 # x center of the figure
    figure.yO = 0.0 # y center of the figure
    figure.zO = 0.0 # z center of the figure
    # Generate the path
    builtFlag = figure.buildPoses(request.figure.figure_id)
    # Sending the response
    if not builtFlag:
      response.poses.poses = figure.poses.poses
      self.get_logger().info('Path generated')
      return response
    else:
      self.get_logger().info(builtFlag)
      self.get_logger().info('Path not generated')
      return response

def main(args=None):
  rclpy.init(args=args)
  path_gen_node = PathGen()
  rclpy.spin(path_gen_node)
  path_gen_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()