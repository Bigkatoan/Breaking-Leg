import torch
import torch.nn as nn
import torch.nn.functional as F
# import torchvision

class encoder(nn.Module):
    def __init__(self, feature_out=1024):
        super().__init__()
        self.feature_out = feature_out

        #256, 256
        self.conv1 = nn.Conv2d(7, 16, 3, padding='same') #128
        self.conv2 = nn.Conv2d(16, 32, 3, padding='same') #64
        self.conv3 = nn.Conv2d(32, 64, 3, padding='same') #32
        self.conv4 = nn.Conv2d(64, 128, 3, padding='same') #16
        self.conv5 = nn.Conv2d(128, 256, 3, padding='same') #8
        self.conv6 = nn.Conv2d(256, 512, 3, padding='same') #4
        self.conv7 = nn.Conv2d(512, 1024, 3, padding='same') #2

        self.flat = nn.Flatten()

        self.feature = nn.Linear(4096, feature_out)
        # self.feature = nn.Linear(458752, feature_out)

    def forward(self, x):
        # x = torchvision.transforms.Resize(256)(x)
        x = F.relu(self.conv1(x))
        x = nn.MaxPool2d(2)(x)
        x = F.relu(self.conv2(x))
        x = nn.MaxPool2d(2)(x)
        x = F.relu(self.conv3(x))
        x = nn.MaxPool2d(2)(x)
        x = F.relu(self.conv4(x))
        x = nn.MaxPool2d(2)(x)
        x = F.relu(self.conv5(x))
        x = nn.MaxPool2d(2)(x)
        x = F.relu(self.conv6(x))
        x = nn.MaxPool2d(2)(x)
        x = F.relu(self.conv7(x))
        x = nn.MaxPool2d(2)(x)

        flat = self.flat(x)
        feature = self.feature(flat)
        return feature

class actor(nn.Module):
    def __init__(self, observation_space, action_space):
        super().__init__()
        self.observation_space = observation_space
        self.action_space = action_space

        self.encoder = encoder()
        self.hidden1 = nn.Linear(1024, 512)
        self.hidden2 = nn.Linear(512, 256)
        self.action = nn.Linear(256, self.action_space)
        
    def forward(self, x):
        x = self.encoder(x)
        x = F.relu(self.hidden1(x))
        x = F.relu(self.hidden2(x))
        return F.tanh(self.action(x))

class critic(nn.Module):
    def __init__(self, observation_space, action_space):
        super().__init__()
        self.observation_space = observation_space
        self.action_space = action_space
        self.input_space = 1024 + self.action_space

        self.encoder = encoder()
        self.hidden1 = nn.Linear(self.input_space, 256)
        self.hidden2 = nn.Linear(256, 512)
        self.q = nn.Linear(512, 1)

    def forward(self, observation, action):
        feature = self.encoder(observation)
        x = torch.concat([feature, action], dim=-1)
        x = F.relu(self.hidden1(x))
        x = F.relu(self.hidden2(x))
        return self.q(x)        