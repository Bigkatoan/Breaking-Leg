import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from bl_msg.msg import Control
from bl_msg.srv import Getimage
from cv_bridge import CvBridge
import cv2
import numpy
import torch

from typing import Optional
import gymnasium as gym
rclpy.init()

class update(Node):
    def __init__(self):
        super().__init__('update')
        self.bridge = CvBridge()
        
        self.color = numpy.zeros((256, 256, 3))
        self.depth = numpy.zeros((256, 256, 1))
        
        self.color_subcription = self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.get_color, 1)
        self.depth_subcription = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.get_depth, 1)
        self.control = self.create_publisher(Control, '/breaking_leg/get_control', 1)
        
    def get_color(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.resize(img, (256, 256))
        img = img[:, :, :3]
        self.color = img
        
    def get_depth(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)
        img = cv2.resize(img, (256, 256))
        img = img.reshape((256, 256, 1))
        img = numpy.nan_to_num(img)
        self.depth = img
        
    def snacontrol(self, speed, angle):
        msg = Control()
        msg.speed = speed
        msg.angle = angle
        self.control.publish(msg)
        
class BLEnv(gym.Env):
    def __init__(self, path='/home/tx2/ros2_ws/goals/test.png'):
        # ros2 connection.
        self.ros2 = update()
        rclpy.spin_once(self.ros2)
            
        # goal setup.
        self.goal = cv2.imread(path)
        self.goal = cv2.resize(self.goal, (256, 256))
        
        # setup base environment.
        self.observation_space = gym.spaces.Dict(
            {
                "observation_rgb": gym.spaces.Box(0, 255, shape=(256, 256, 3), dtype=float),
                "observation_depth": gym.spaces.Box(0, 255, shape=(256, 256, 1), dtype=float),
                "goal": gym.spaces.Box(0, 255, shape=(256, 256, 3), dtype=float),
            }
        )
        self.action_space = gym.spaces.Box(-1, 1, shape=(2,), dtype=float)
    
    def get_color(self):
        img = self.ros2.color
        img = cv2.resize(img, (256, 256))
        return img[:, :, :3][:, :, ::-1]
    
    def get_depth(self):
        img = self.ros2.depth
        img = cv2.resize(img, (256, 256))
        return img.reshape((256, 256, 1))
    
    def _get_obs(self):
        rclpy.spin_once(self.ros2)
        self.color = self.get_color()
        self.depth = self.get_depth()
        return {"observation_rgb": self.color, "observation_depth": self.depth, "goal": self.goal}
    
    def _get_info(self):
        c = self.color
        d = self.goal
        # sim = numpy.sum((c - d)**2)**.5
        return {"similarity": 1}
        # return {"sim": 1}
    
    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        observation = self._get_obs()
        info = self._get_info()
        return observation, info
    
    def step(self, action):
        speed = int(action[0] * 100)
        angle = int(action[1] * 90)
        self.ros2.snacontrol(speed, angle)
        for i in range(3):
            rclpy.spin_once(self.ros2)
        terminated = 0
        truncated = False
        reward = -1
        observation = self._get_obs()
        info = self._get_info()
        return observation, reward, terminated, truncated, info
    
gym.register(
    id="BLEnv-v0",
    entry_point=BLEnv
)