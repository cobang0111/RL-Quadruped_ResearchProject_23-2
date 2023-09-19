#custom_env of cart-pole
import gymnasium as gym
import numpy as np
from time import sleep
import math, random
import matplotlib.pyplot as plt
from tqdm import tqdm
import warnings
from typing import Tuple
from sklearn.preprocessing import KBinsDiscretizer

warnings.filterwarnings('ignore', category=UserWarning)

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
env = CustomCartPoleEnv(gym.make('CartPole-v1', render_mode = None))
#env = gym.make('CartPole-v1', render_mode = None)
# Visualize the environment
#policy =lambda obs: 1

#for _ in range(5):
    #obs = env.reset()
    #for _ in range(80):
        #actions = policy(obs)
        #obs, reward, terminated, truncated, info = env.step(actions)
        #env.render()
        #time.sleep(0.05)
        #if terminated or truncated:
            #obs, info = env.reset()

#env.close()

policy = lambda _,__,___, tip_velocity : int(tip_velocity > 0)

# Q-Learning
n_bins = (6, 12)
lower_bounds = [env.observation_space.low[2], -math.radians(50)]
upper_bounds = [env.observation_space.high[2], math.radians(50)]

def discretizer(__, ___, angle, pole_velocity) -> Tuple[int,...]:
    est = KBinsDiscretizer(n_bins=n_bins, encode='ordinal', strategy='uniform')
    est.fit([lower_bounds, upper_bounds])
    return tuple(map(int,est.transform([[angle, pole_velocity]])[0]))

# Initialize the Q-table
Q_table = np.zeros(n_bins + (env.action_space.n,))
Q_table.shape

# Creating policy function uses the Q-table and selects highest Q-value
def policy(state: tuple):
    return np.argmax(Q_table[state])  # exploit

# Update the Q-table using the Bellman equation
def new_Q_value(reward:float, new_state:tuple, discount_factor = 1) -> float:
    #Temperal difference for updating Q-value of state-action pair
    # bellman equation?
    future_optimal_value = np.max(Q_table[new_state])
    learned_value = reward + discount_factor * future_optimal_value
    return learned_value

# Decaying learning rate
# Adaptive Learning of Learning rate
def learning_rate(n: int, min_rate = 0.01) -> float:
    return max(min_rate, min(1.0, 1.0 - math.log10((n + 1) / 25)))

# Decaying exploration rate
def exploration_rate(n: int, min_rate = 0.2) -> float:
    return max(min_rate, min(1.0, 1.0 - math.log10((n  + 1) / 25)))

# Training
n_episodes = 600
queue = []
queue2 = []
queue_index = []
best_episode = None
best_reward = float('-inf')

for e in tqdm(range(n_episodes)):
    # Siscretize state into buckets
    current_state = discretizer(*env.reset()[0])
    total_reward = 0
    episode = []
    done = False
    while not done:
        # Policy action
        action = policy(current_state)
        # insert random action
        if np.random.random() < exploration_rate(e):
            action = env.action_space.sample()
        #increment enviroment
        obs, reward, terminated, truncated, _ = env.step(action)
        #print(obs[0])
        done = terminated or truncated
        new_state = discretizer(*obs)
        total_reward += reward
        episode.append((obs, action)) 

        # Update Q-Table
        lr = learning_rate(e)
        learnt_value = new_Q_value(reward, new_state)
        old_value = Q_table[current_state][action]
        Q_table[current_state][action] = (1 - lr) * old_value + lr * learnt_value

        current_state = new_state

        if terminated or truncated:
            obs, info = env.reset()

    #print(round(total_reward, 2))
    if total_reward > best_reward:
        #print(f"Episode {e} - New Max Reward {round(total_reward,2)}")
        queue.append(total_reward)
        queue_index.append(e)
        best_reward = total_reward
        best_episode = episode    
    
    queue2.append(total_reward)

        
# Visualization
plt.plot(queue_index, queue)
plt.show()

plt.plot(queue2)
plt.show()

print(best_episode)

env.close()
env = CustomCartPoleEnv(gym.make('CartPole-v1', render_mode = 'human'))
#env = gym.make('CartPole-v1', render_mode = 'human')
obs, info = env.reset()

for i in range(10):
    current_state = discretizer(*env.reset()[0])
    done = False
    while not done:
        # Policy action
        action = np.argmax(Q_table[current_state])
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        new_state = discretizer(*obs)
        total_reward+=reward

        current_state = new_state
        if terminated or truncated:
            obs, info = env.reset()
        
        env.render()

env.close()
