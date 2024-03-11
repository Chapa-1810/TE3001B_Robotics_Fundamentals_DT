#!/usr/bin/env python3

## listens to a tf topic and publisher markers to visualize the path of the end effector in rviz
import rclpy
from rclpy.node import Node
import tf2_ros
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose

class EefDraw(Node):
    def __init__(self):
        super().__init__('eef_draw')    
        self.get_logger().info('Eef Draw node started')
        self.tfBuffer = tf2_ros.Buffer()
        self.get_logger().info('Waiting for transform')
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.get_logger().info('Transform received')
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        # a stack of markers to visualize the path of the end effector, deleting the oldest marker and adding a new one
        self.marker_array = MarkerArray()
        self.marker_array_max_size = None # set as none for infinite size
        self.marker_rgb = [0.7, 0.0, 0.7] # 0.0 - 1.0
        self.marker_alpha = 1.0 # transparency
        self.marker_size_xyz = [0.01, 0.01, 0.01] # [x, y, z]
        marker_types = [Marker.LINE_STRIP, Marker.CUBE, Marker.SPHERE, Marker.POINTS]
        self.marker_type = marker_types[0]
        self.marker_array.markers.append(Marker())
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tfBuffer.lookup_transform('world', 'link_eef', rclpy.time.Time())
            self.create_marker(transform)
            if self.marker_array.markers[0] is not None: print('Publishing marker: ' + str(self.marker_array.markers[0]))
            self.marker_publisher.publish(self.marker_array)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info('Exception: ' + str(e))
            pass
    
    def create_marker(self, transform):
        marker = Marker()
        if self.marker_type in [Marker.LINE_STRIP, Marker.POINTS]:
            marker = self.marker_array.markers[0]
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 0
            marker.type = self.marker_type
            marker.action = marker.ADD
            #set the pose of the marker at the frame of frame_id, or the base frame
            marker_pose = Pose()
            marker.scale.x = self.marker_size_xyz[0]
            marker.scale.y = self.marker_size_xyz[1]
            marker.scale.z = self.marker_size_xyz[2]
            marker.color.a = self.marker_alpha
            marker.color.r = self.marker_rgb[0]
            marker.color.g = self.marker_rgb[1]
            marker.color.b = self.marker_rgb[2]
            newPoint = Point()
            newPoint.x = transform.transform.translation.x
            newPoint.y = transform.transform.translation.y
            newPoint.z = transform.transform.translation.z  
            marker.points.append(newPoint)
            if self.marker_array_max_size is not None:
                if len(marker.points) > self.marker_array_max_size:
                    marker.points.pop(0)
            self.marker_array.markers[0] = marker
            
        else:
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = len(self.marker_array.markers)
            marker.type = self.marker_type
            marker.action = marker.ADD
            marker.pose.position.x = transform.transform.translation.x
            marker.pose.position.y = transform.transform.translation.y
            marker.pose.position.z = transform.transform.translation.z
            marker.pose.orientation.x = transform.transform.rotation.x
            marker.pose.orientation.y = transform.transform.rotation.y
            marker.pose.orientation.z = transform.transform.rotation.z
            marker.pose.orientation.w = transform.transform.rotation.w
            marker.scale.x = self.marker_size_xyz[0]
            marker.scale.y = self.marker_size_xyz[1]
            marker.scale.z = self.marker_size_xyz[2]
            marker.color.a = self.marker_alpha
            marker.color.r = self.marker_rgb[0]
            marker.color.g = self.marker_rgb[1]
            marker.color.b = self.marker_rgb[2]
            self.marker_array.markers.append(marker)
            if self.marker_array_max_size is not None:
                if len(self.marker_array.markers) > self.marker_array_max_size:
                    self.marker_array.markers.pop(0)
            id = 0
            for marker in self.marker_array.markers:
                marker.id = id
                id += 1
    
def main(args=None):
    rclpy.init(args=args)
    eef_draw = EefDraw()
    rclpy.spin(eef_draw)
    eef_draw.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    