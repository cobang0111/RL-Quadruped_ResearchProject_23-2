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
pybullet.setTimeStep(1/240) #Time step
#FLAG = (pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | pybullet.URDF_USE_SELF_COLLISION)
#FLAG = pybullet.URDF_USE_SELF_COLLISION
dir = r"C:\Users\rkobb\Desktop\2023-2\Research_Project_I\solidworks\sgmc_231006v8\urdf\sgmc_231006v8.urdf"
robot = pybullet.loadURDF(dir, [0,0,0], useFixedBase=1, useMaximalCoordinates=False)

#Set collision
collisionFilterGroup = 1
collisionFilterMask = 1
pybullet.setCollisionFilterGroupMask(robot, -1, collisionFilterGroup, collisionFilterMask)
num_joints = pybullet.getNumJoints(robot)
for i in range(num_joints):
    pybullet.setCollisionFilterGroupMask(robot, i, collisionFilterGroup, collisionFilterMask)

#pybullet.setCollisionFilterPair(robot, robot, -1, 0, 1)
#pybullet.setCollisionFilterPair(robot, robot, -1, 1, 1)
#pybullet.setCollisionFilterPair(robot, robot, -1, 2, 1)
#pybullet.setCollisionFilterPair(robot, robot, -1, 3, 1)

#pybullet.setCollisionFilterPair(robot, robot, 0, 1, 1)
#pybullet.setCollisionFilterPair(robot, robot, 0, 2, 1)
#pybullet.setCollisionFilterPair(robot, robot, 0, 3, 1)

#pybullet.setCollisionFilterPair(robot, robot, 1, 2, 1)
#pybullet.setCollisionFilterPair(robot, robot, 1, 3, 1)

#pybullet.setCollisionFilterPair(robot, robot, 2, 3, 1)


#print(pybullet.getBasePositionAndOrientation(0))
#print(pybullet.getJointInfo(robot, 0))

for joint_index in range(pybullet.getNumJoints(robot)):
    pybullet.changeVisualShape(robot, joint_index, rgbaColor=[1, 0, 0, 1]) 
    pybullet.setJointMotorControl2(robot, joint_index, pybullet.VELOCITY_CONTROL, force = 0)
    pybullet.changeDynamics(robot, joint_index, lateralFriction=0.2376545)
    #pybullet.resetBasePositionAndOrientation(robot, [0, 0, 0.3], [1, 1, 1, 1])  # 위치 및 방향 재설정

# Get the joint indices
#joint_indices = [j for j in range(pybullet.getNumJoints(robot))]

# Set the motor control for all joints
#pybullet.setJointMotorControl2(robot, 1, pybullet.VELOCITY_CONTROL, targetVelocity=+5)
#pybullet.setJointMotorControl2(robot, 4, pybullet.VELOCITY_CONTROL, targetVelocity=+4)


pybullet.setGravity(0, 0, -9.81) # define x,y,z gravity constants
pybullet.setRealTimeSimulation(1)
# Run the simulation
while (pybullet.isConnected()):
    time.sleep(1/240)
    #pybullet.setGravity(0, 0, -9.81)
    