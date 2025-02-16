import cv2
import numpy
import torch
from collections import deque
import random

class HER:
    def __init__(self, maxlen=10000, batch_size=32):
        """
        Hindsight Memory Buffer implementation.
        by: Tran Dang An from FPT University.
        
        path: path to MEMORY folder.
        """
        self.maxlen = maxlen
        self.batch_size = batch_size
        self.n = 4
        
        self.trajectory = []
        self.buffer = deque(maxlen=maxlen)
        self.flag = False
        
    def temp_save(self, s, a, r, s_, t, g):
        """
        save trajectory to temporary buffer.
        """
        # save normal.
        self.trajectory.append([s, a, r, s_, t, g])
        
    def save(self):
        """
        save trajectory to semi-longterm buffer.
        """
        if self.trajectory == []:
            return False
        new_goal = self.trajectory[-1][5]
        new_goal_trajectory = self.trajectory.copy()
        for i in range(len(new_goal_trajectory)):
            new_goal_trajectory[i][5] = new_goal
            new_goal_trajectory[i][2] = 0
            self.buffer.append(self.trajectory[i])
            self.buffer.append(new_goal_trajectory[i])
        self.trajectory = []
        self.flag = True
        return True
    
    def get_batch(self):
        if not self.flag:
            return
        S, A, R, S_, T, G = random.sample(self.buffer, k=self.batch_size)
        S = torch.tensor([val[0] for val in batch]).view(self.batch_size, self.n, 256, 256).to(torch.float32)
        A = torch.tensor([val[1] for val in batch]).view(self.batch_size, -1).to(torch.float32)
        R = torch.tensor([val[2] for val in batch]).view(self.batch_size, -1).to(torch.float32)
        S_ = torch.tensor([val[3] for val in batch]).view(self.batch_size, self.n, 256, 256).to(torch.float32)
        T = torch.tensor([val[4] for val in batch]).view(self.batch_size, -1).to(torch.float32)
        G = torch.tensor([val[5] for val in batch]).view(self.batch_size, 3, 256, 256).to(torch.float32)
        S = torch.concat([S, G], dim=1)
        return S, A, R, S_, T