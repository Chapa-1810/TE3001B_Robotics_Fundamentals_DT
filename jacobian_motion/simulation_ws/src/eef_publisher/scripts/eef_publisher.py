#!/usr/bin/env python3

## listens to a tf topic and publisher markers to visualize the path of the end effector in rviz
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped

class EefPublisher(Node):
    def __init__(self):
        super().__init__('eef_publisher')    
        self.get_logger().info('Eef Draw node started')
        self.tfBuffer = tf2_ros.Buffer()
        self.get_logger().info('Waiting for transform')
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.get_logger().info('Transform received')
        self.eef_publisher = self.create_publisher(PoseStamped, 'eef_pose', 10)
        self.PUBLISH_FREQUENCY = 100
        self.create_timer(1/self.PUBLISH_FREQUENCY, self.timer_callback)

    def timer_callback(self):
        try:
            transform_pose = self.tfBuffer.lookup_transform('base', 'link_eef', rclpy.time.Time())
            self.eef_publisher.publish(transform_pose)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info('Exception: ' + str(e))
            pass
        
    def get_pose(self, link, frame) -> PoseStamped:
        transform = self.tfBuffer.lookup_transform(link, frame, rclpy.time.Time())
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation.x = transform.transform.rotation.x
        pose.pose.orientation.y = transform.transform.rotation.y
        pose.pose.orientation.z = transform.transform.rotation.z
        pose.pose.orientation.w = transform.transform.rotation.w
        return pose
        
def main(args=None):
    rclpy.init(args=args)
    eef_publisher = EefPublisher()
    rclpy.spin(eef_publisher)
    eef_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    