import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class Setpoint(Node):
    def __init__(self):
        super().__init__('set_point_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('type_int', rclpy.Parameter.Type.INTEGER), 
                ('time_float', rclpy.Parameter.Type.DOUBLE),
                ('offset_float', rclpy.Parameter.Type.DOUBLE), 
                ('frequency_float', rclpy.Parameter.Type.DOUBLE), 
                ('amplitude_float', rclpy.Parameter.Type.DOUBLE)
            ]
        )
        # self.signal_publisher = self.create_publisher(Float32, 'pwm_duty_cycle', 10)
        # signal_timer_period = 0.05
        # self.signal_timer = self.create_timer(signal_timer_period, self.signal_timer_callback)
        # self.signal_sub = self.create_subscription(Float32, 'angular_speed', self.angularS_callback, 10)
        # self.get_logger().info('Signal generator node initialized')
        # self.msg_signal = Float32()
        self.current_w = 0
        self.desired_w = 0
        self.kp = 0.5
        self.ki = 0.5
        self.acum_error = 0
        self.max_duty_cycle = 0
        self.setpoint_publisher_ = self.create_publisher(Float32, '/setpoint', 10)
        self.controller_subscriber_ = self.create_subscription(Float32, '/controller', self.controller_callback, 10)
        self.signal_subscriber_ = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.timer_ = 0.05
        self.setpoint_timer_ = self.create_timer(self.timer_, self.setpoint_callback)

        self.get_logger().info('Signal generator node initialized')

    def controller_callback(self,msg):
        self.current_w = msg.data
        self.get_logger().info('Current angular speed: {}'.format(self.current_w))

    def signal_callback(self, msg):
        self.desired_w = msg.data
        self.get_logger().info('Desired angular speed: {}'.format(self.desired_w))

    def setpoint_callback(self):
        msg_setpoint = Float32()
        error = self.desired_w - self.current_w
        output = self.kp * error + self.ki * self.timer_ * (error + self.acum_error)
        self.acum_error += error

        output = output / self.max_duty_cycle
        output = output * (abs(error)/error)

        msg_setpoint.data = output
        self.setpoint_publisher_.publish(output)
        self.get_logger().info('Setpoint published: {}'.format(output))
    
    
        
def main(args=None):
    rclpy.init(args=args)
    m_p = Setpoint()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    