import pybullet
import pybullet_data
import numpy as np
import matplotlib as plt
from tqdm import tqdm
import math
from typing import Tuple
import gym
from gym import spaces
import imageio_ffmpeg
from stable_baselines3.sac.policies import MlpPolicy
from stable_baselines3 import SAC
import time

# Simulation & Environment
pybullet.connect(pybullet.GUI) #Open pybulle simulation
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setTimeStep(1/400) #Time step

#FLAG = (pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | pybullet.URDF_USE_SELF_COLLISION)
FLAG = pybullet.URDF_USE_SELF_COLLISION

plane = pybullet.loadURDF("plane.urdf")
robot = pybullet.loadURDF("mini_cheetah/mini_cheetah.urdf", [0, 0, 0.5], flags=FLAG)


#pybullet.changeDynamics(robot, 0, lateralFriction=0.5)
for joint_index in range(pybullet.getNumJoints(robot)):
    pybullet.changeVisualShape(robot, joint_index, rgbaColor=[1, 0, 0, 1]) 
    pybullet.setJointMotorControl2(robot, joint_index, pybullet.VELOCITY_CONTROL, force = 0)
    pybullet.changeDynamics(robot, joint_index, lateralFriction=0.8)
    #pybullet.resetBasePositionAndOrientation(robot, [0, 0, 0.3], [1, 1, 1, 1])  # 위치 및 방향 재설정

pybullet.setGravity(0, 0, -9.81) # define x,y,z gravity constants
pybullet.resetDebugVisualizerCamera(cameraTargetPosition=[0, 0.25, 0], cameraDistance=2, cameraYaw=81, cameraPitch=-42.6)
pybullet.setRealTimeSimulation(1)
# Run the simulation
while (pybullet.isConnected()):
    time.sleep(1/400)
    
    