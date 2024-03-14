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
        self.current_w = 0.0
        self.desired_w = 0.0
        self.kp = 0.5
        self.ki = 0.6
        self.acum_error = 0
        self.min_output = 0.25
        self.min_speed = 0.5
        self.max_acum_error = 2
        self.max_duty_cycle = 1
        self.setpoint_publisher_ = self.create_publisher(Float32, '/setpoint', 10)
        self.controller_subscriber_ = self.create_subscription(Float32, '/angular_speed', self.controller_callback, 10)
        self.signal_subscriber_ = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.desired_speed_publisher = self.create_publisher(Float32, '/desired_speed', 10)
        self.timer_ = 0.02
        self.setpoint_timer_ = self.create_timer(self.timer_, self.setpoint_callback)

        self.get_logger().info('Signal generator node initialized')

    def controller_callback(self,msg):
        self.current_w = msg.data
        #self.get_logger().info('Current angular speed: {}'.format(self.current_w))

    def signal_callback(self, msg):
        self.desired_w = msg.data
        #self.get_logger().info('Desired angular speed: {}'.format(self.desired_w))

    def setpoint_callback(self):
        desired_speed_msg = Float32()
        desired_speed_msg.data = self.desired_w
        self.desired_speed_publisher.publish(desired_speed_msg)
        if abs(self.desired_w) < self.min_speed:
            msg_setpoint = Float32()
            msg_setpoint.data = 0.0
            self.setpoint_publisher_.publish(msg_setpoint)
            return
        
        msg_setpoint = Float32()
        error = self.desired_w - self.current_w
        output = self.kp * error + self.ki * self.acum_error
        self.acum_error += error * self.timer_
        
        if abs(output) < self.min_output and abs(output) > 0:
            output = self.min_output if self.desired_w > 0 else -self.min_output
        else:
            output = output / self.max_duty_cycle
        
        if (output > 0 and self.desired_w < 0) or (output < 0 and self.desired_w > 0):
            self.acum_error = 0
            output = 0.0
        
        msg_setpoint.data = output
        self.setpoint_publisher_.publish(msg_setpoint)
        self.get_logger().info('Setpoint published: {}'.format(output))
        # if abs(self.acum_error) > self.max_acum_error:
        #     self.acum_error = self.max_acum_error if self.desired_w > 0 else -self.max_acum_error
        
        # print(f"PID INFO: error: {error}, output: {output}, acum_error: {self.acum_error}")
        # print("--------------------")
        # if abs(output) < self.min_output and abs(output) > 0:
        #     output = self.min_output if self.desired_w > 0 else -self.min_output
        # else:
        #     output = output / self.max_duty_cycle
        #     #output = output if self.desired_w > 0 else -output
        # # ensure desired speed is the same sign as the output
        # if self.desired_w > 0:
        #     if not output > 0:
        #         output = -output
        # else:
        #     if not output < 0:
        #         output = -output
        # msg_setpoint.data = output
        # self.setpoint_publisher_.publish(msg_setpoint)
        # self.get_logger().info('Setpoint published: {}'.format(output))
        
def main(args=None):
    rclpy.init(args=args)
    m_p = Setpoint()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    