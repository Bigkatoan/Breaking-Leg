import rclpy
from rclpy.node import Node
from rplidar import RPLidar
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import os
import numpy

class RPlidarA2(Node):
    def __init__(self):
        super().__init__('lidar')
        self.lidar = RPLidar('/dev/ttyUSB0')
        
        self.info = self.lidar.get_info()
        self.health = self.lidar.get_health()
        
        self.get_logger().info(f"""
        info:
        {self.info}
        health:
        {self.health}
        """)
        
        self.msg = LaserScan()
        self.msg.header.frame_id = 'laser'
        self.msg.angle_min = -numpy.pi
        self.msg.angle_max = numpy.pi
        self.msg.angle_increment = numpy.pi*2/360
        self.msg.range_min = 150/1000
        self.msg.range_max = 5000/1000
        self.distances = numpy.ones((360, )) * (5000 / 1000)
        
        self.publisher_ = self.create_publisher(msg_type=LaserScan, topic='/lidar', qos_profile=1)
        
        self.get_scan()
        
    def get_scan(self):
        # millimet, degree
        i = 1
        for scan in self.lidar.iter_measures():
            new_scan, strg, angle, distance = scan
            self.distances[max(int(angle) - 1, 0)] = min(distance/1000, 5000/1000)
            i += 1
            if i % 300 == 0:
                self.msg.header.stamp = rclpy.time.Time().to_msg()
                self.msg.ranges = self.distances.tolist()
                self.publisher_.publish(self.msg)
                i = 1
def main(arg=None):
    rclpy.init()
    node = RPlidarA2()
    rclpy.spin(node)
    rclpy.shutdown