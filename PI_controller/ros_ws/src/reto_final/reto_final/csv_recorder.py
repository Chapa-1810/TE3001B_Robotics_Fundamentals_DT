import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32
import time
import os

class CSVRecorder(Node):
    def __init__(self):
        super().__init__('csv_recorder_node')
        
        self.csv_file_path = "data.csv"
        if os.path.exists(self.csv_file_path):
            os.remove(self.csv_file_path)
        self.get_logger().info(f'CSV file created on {self.csv_file_path}')
        self.csv_file = open(self.csv_file_path, 'w')
        self.csv_file.write('Time,Desired Speed,Actual Speed,PWM Setpoint\n')
        
        self.desired_speed = 0
        self.actual_speed = 0
        self.pwm_setpoint = 0
        self.desired_speed_sub = self.create_subscription(Float32, '/desired_speed', self.desired_speed_callback, 10)
        self.actual_speed_sub = self.create_subscription(Float32, '/angular_speed', self.actual_speed_callback, 10)
        self.pwm_setpoint_sub = self.create_subscription(Float32, '/setpoint', self.pwm_setpoint_callback, 10)
        
        self.csv_timer_period = 0.02
        self.csv_timer = self.create_timer(self.csv_timer_period, self.csv_timer_callback)
        
    def csv_timer_callback(self):
        self.csv_file.write(f'{time.time()},{self.desired_speed},{self.actual_speed},{self.pwm_setpoint}\n')
        
    def desired_speed_callback(self, msg):
        self.desired_speed = msg.data
    
    def actual_speed_callback(self, msg):
        self.actual_speed = msg.data
    
    def pwm_setpoint_callback(self, msg):
        self.pwm_setpoint = msg.data
        
def main(args=None):
    rclpy.init(args=args)
    csv_recorder = CSVRecorder()
    rclpy.spin(csv_recorder)
    csv_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    