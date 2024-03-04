#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from gtts import gTTS
from pygame import mixer

class Speak(Node):
    def __init__(self) -> None:
        super().__init__('speak')
        self.speaking_sub = self.create_subscription(String, 'speak_text', self.listener_callback, 10)
        self.speaking_pub = self.create_publisher(Bool, 'speaking', 10)
        self.speaking_pub.publish(Bool(data=False))
        self.get_logger().info('Speak node is running')
        
        

    def listener_callback(self, msg):
        text = msg.data
        self.speak(text)

    def speak(self, text):
        self.get_logger().info(f"Speaking: {text}")
        tts = gTTS(text)
        tts.save('voice.mp3')
        mixer.init()
        mixer.music.load('voice.mp3')
        mixer.music.play()
        self.speaking_pub.publish(Bool(data=True))
        while mixer.music.get_busy():
            pass
        self.speaking_pub.publish(Bool(data=False))
        mixer.quit()
        self.get_logger().info('Speaking finished')
        

if __name__ == '__main__':
    rclpy.init()

    speak = Speak()

    rclpy.spin(speak)

    speak.destroy_node()
    rclpy.shutdown()
