import rclpy
from rclpy.node import Node
from figure_msgs.msg import Figure
from figure_msgs.srv import path_generator

class PathGen(Node):
  def __init__(self):
    super().__init__('pathgen')
    self.srv = self.create_service(path_generator, 'add_two_ints', self.path_generation_callback)
  
  def path_generation_callback(self, request, response):
    request.figure
    response.poses 
    self.get_logger().info('Path generated')

    return response


def main(args=None):
  rclpy.init(args=args)

  path_gen_node = PathGen()

  rclpy.spin(path_gen_node)

  path_gen_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()