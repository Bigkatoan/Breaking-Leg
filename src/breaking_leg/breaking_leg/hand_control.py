import rclpy
from rclpy.node import Node
from bl_msg.msg import Control
import cv2
import numpy

class get_control(Node):
    def __init__(self):
        super().__init__('get_control')
        self.speed = 0
        self.angle = 0
        self.publisher_ = self.create_publisher(Control, 
                                                '/breaking_leg/get_control',
                                                1)
        self.control()

    def config(self, key):
        if key == ord('a'):
            self.angle = max(self.angle - 5, -90)
        if key == ord('d'):
            self.angle = min(self.angle + 5, 90)
        if key == ord('w'):
            self.speed = min(self.speed + 5, 100)
        if key == ord('s'):
            self.speed = max(self.speed - 5, -100)
        if key == ord('q'):
            self.speed = 0
            self.angle = 0
        if key == ord('e'):
            self.angle = 0

    def control(self):
        while True:
            cv2.imshow('canvas', numpy.zeros((640, 320)))
            key=cv2.waitKey(1)
            self.config(key)
            msg = Control()
            msg.speed = self.speed
            msg.angle = self.angle
            self.publisher_.publish(msg)

def main(arg=None):
    rclpy.init()
    node = get_control()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()