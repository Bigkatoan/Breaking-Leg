import rclpy
from rclpy.node import Node
from bl_msg.msg import Control
from sensor_msgs.msg import Image
import cv2
import numpy
from cv_bridge import CvBridge
from breaking_leg.utils.YOLO_Models import YOLO

#640 640
class tracking(Node):
    def __init__(self):
        super().__init__('tracking')
        self.angle = 0
        self.get_logger().info("load model")
        self.model = YOLO('/home/tx2/ros2_ws/models/yolo/yolo11m.torchscript', device='cuda')
        self.get_logger().info("load model complete")
        self.subcriber_ = self.create_subscription(Image, 
                                                   '/zed/zed_node/left_raw/image_raw_color', 
                                                   self.detect, 1)
        self.publisher_ = self.create_publisher(Control, '/breaking_leg/get_control', 1)
        self.bridge = CvBridge()
        
    def get_key(self, key):
        if key == ord('q'):
            self.destroy_node()
    
    def detect(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        img = img[:, :, :3]
        pred = self.model.predict(img)
        """
        pred:
            list: 6 item
            first 4: bbox x1, y1, x2, y2
            5: confidence
            6: class (0: person)
        """
        img = self.model.draw(img, pred)
        cv2.imshow('camera', img)
        
        msg = Control()
        max_s = -9999
        cord = []
        idx = 0
        flag = False
        for i in range(len(pred)):
            val = pred[i]
            self.get_logger().info(f"{val}")
            if val[5] != 0:
                continue
            flag = True
            s = abs(val[0] - val[2]) * abs(val[1] - val[3])
            if s > max_s:
                max_s = s
                idx = i
                cord = val
        if flag:
            track = pred[idx]
            val = cord
            xc = (val[0] + abs(val[0] - val[2])/2 - 320)/320
            xc = xc * 90
            if max_s > 40000:
                msg.speed = 0
            else:
                msg.speed = 50
            msg.angle = int(xc)
            self.angle = int(xc)
            self.publisher_.publish(msg)
        else:
            msg.speed = 0
            msg.angle = self.angle
            self.publisher_.publish(msg)
            
        key = cv2.waitKey(1)
        self.get_key(key)
        
def main(arg=None):
    rclpy.init()
    node = tracking()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
