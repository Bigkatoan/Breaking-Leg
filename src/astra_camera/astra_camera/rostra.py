import rclpy
from rclpy.node import Node
from modules.motra import Camera
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
import numpy
import struct

class astra(Node):
    def __init__(self):
        super().__init__('astra')
        self.cam = Camera()
        self.bridge = CvBridge()
        self.rgb_publisher_ = self.create_publisher(Image, '/astra_rgb', 1)
        self.depth_publisher_ = self.create_publisher(PointCloud2, '/astra_depth', 1)
        self.timer_ = self.create_timer(1/30, self.publish)
        self.height = 480//2
        self.width = 640//2
        
    def depth_to_pointcloud(self, depth_image, camera_intrinsics={
        'fx': 525./2, 'fy': 525./2, 'cx': 319.5/2, 'cy':239/2
    }):
        """
        Convert a depth image into a point cloud.

        Parameters:
        - depth_image: The depth image (2D NumPy array).
        - camera_intrinsics: A dictionary with keys:
            * fx: Focal length in x direction
            * fy: Focal length in y direction
            * cx: Principal point x
            * cy: Principal point y

        Returns:
        - points: A NumPy array of shape (N, 3) containing 3D points.
        """
        height, width= self.width, self.height
        fx, fy = camera_intrinsics["fx"], camera_intrinsics["fy"]
        cx, cy = camera_intrinsics["cx"], camera_intrinsics["cy"]

        # Create mesh grid for pixel coordinates
        u, v = numpy.meshgrid(numpy.arange(self.width), numpy.arange(self.height))

        # Calculate 3D coordinates
        z = depth_image.astype(numpy.float32).reshape(self.height, self.width)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        # Stack into N x 3 point cloud480
        points = numpy.stack((x.flatten(), y.flatten(), z.flatten()), axis=-1)
        return points
    
    def publish(self):
        depth, img = self.cam.get_depth_and_color()
        
        img_msg = self.bridge.cv2_to_imgmsg(img)
        img_msg.header.frame_id = 'rgb'
        self.rgb_publisher_.publish(img_msg)
        
        points = self.depth_to_pointcloud(depth/1000)
        points_flat = numpy.array(points, dtype=numpy.float32).flatten()

        # Define PointCloud2 message
        header = Header()
        header.frame_id = 'depth'
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pointcloud_msg = PointCloud2(
            header=header,
            height=480//2,  # Unordered point cloud (height=1)
            width=640//2,
            fields=fields,
            is_bigendian=False,
            point_step=12,  # 3 fields * 4 bytes/field
            row_step=12 * len(points),
            data=struct.pack(f'{len(points_flat)}f', *points_flat),
            is_dense=True
        )
        self.depth_publisher_.publish(pointcloud_msg)
        
def main(arg=None):
    rclpy.init()
    node = astra()
    rclpy.spin(node)
    rclpy.shutdown()