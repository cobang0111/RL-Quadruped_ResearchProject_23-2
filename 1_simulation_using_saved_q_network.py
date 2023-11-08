import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import math
import random
import warnings
warnings.filterwarnings('ignore', category=UserWarning)


class DQN(nn.Module):
    def __init__(self, n_observations, n_actions):
        super(DQN, self).__init__()
        self.layer1 = nn.Linear(n_observations, 128)
        self.layer2 = nn.Linear(128, 128)
        self.layer3 = nn.Linear(128, n_actions)

    # 최적화 중에 다음 행동을 결정하기 위해서 하나의 요소 또는 배치를 이용해 호출
    def forward(self, x):
        x = F.relu(self.layer1(x))
        x = F.relu(self.layer2(x))
        return self.layer3(x)

class CustomCartPoleEnv(gym.Env):
    def __init__(self, env):
        self.env = env
        self.action_space = self.env.action_space
        self.observation_space = self.env.observation_space

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        theta = obs[2]
        if 10  <= np.rad2deg(theta) <= 20:
            reward += 5.01 # pole 각도가 10~20도일 때 big reward
        else:
            reward -= 0.9 # 그 이외에는 small penalty
        # Terminated condition
        if abs(theta) < np.radians(50) :
            terminated = False
        if abs(obs[0]) <= 2.4:
            terminated = False
        elif abs(obs[0]) > 2.4:
            terminated = True
        return obs, reward, terminated, truncated, info
            
    def reset(self):
        return self.env.reset()

    def render(self):
        return self.env.render()

# 사용 예제
env = CustomCartPoleEnv(gym.make('CartPole-v1', render_mode = 'human'))

# Q Network 생성 및 로드
state_size = env.observation_space.shape[0]
action_size = env.action_space.n
q_network = DQN(state_size, action_size)
q_network.load_state_dict(torch.load('q_network.pth', map_location=torch.device('cpu')))

# 시뮬레이션 시작
for _ in range(10):
    state, info = env.reset()
    done = False
    while not done:
        env.render()
        state_tensor = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
        action_values = q_network(state_tensor)
        action = torch.argmax(action_values).item()
        state, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated

env.close()
