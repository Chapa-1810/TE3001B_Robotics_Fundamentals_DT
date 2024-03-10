import numpy as np
import rclpy
from scipy import signal
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32

class My_Publisher(Node):
    def __init__(self):
        super().__init__('signal_generator_node') #Change node name as well
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
        self.signal_publisher = self.create_publisher(Float32, 'pwm_duty_cycle', 10)
        signal_timer_period = 0.05
        self.signal_timer = self.create_timer(signal_timer_period, self.signal_timer_callback)
        self.get_logger().info('Signal generator node initialized')
        self.msg_signal = Float32()
        
    def signal_timer_callback(self):
        
        type = self.get_parameter('type_int').get_parameter_value().integer_value
        frequency = self.get_parameter('frequency_float').get_parameter_value().double_value
        amplitude = self.get_parameter('amplitude_float').get_parameter_value().double_value
        time = self.get_parameter('time_float').get_parameter_value().double_value
        offset = self.get_parameter('offset_float').get_parameter_value().double_value
        if(type == 0):
            result = amplitude + offset
        elif(type == 1):
            result = amplitude*np.sin(2*np.pi*time*frequency) + offset
        elif(type == 2):
            result = amplitude*signal.square(2*np.pi*time*frequency) + offset   
        elif(type == 3):
            result = amplitude*signal.sawtooth(2*np.pi*time*frequency) + offset

        self.msg_signal.data = result
        self.signal_publisher.publish(self.msg_signal)
        self.get_logger().info('Signal: {}'.format(self.msg_signal.data))
        param_time = Parameter('time_float', Parameter.Type.DOUBLE, time + 0.05) #Change time
        self.set_parameters([param_time])
        
        
def main(args=None):
    rclpy.init(args=args)
    m_p = My_Publisher()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    