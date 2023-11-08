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

def get_joint_info(robot):
    print('The system has', pybullet.getNumJoints(robot), 'joints')
    num_joints = pybullet.getNumJoints(robot)
    for i in range(num_joints):
        joint_info = pybullet.getJointInfo(robot, i)
        print('Joint number',i)
        print('-------------------------------------')
        print('Joint Index:',joint_info[0])
        print('Joint Name:',joint_info[1])
        print('Joint misc:',joint_info[2:])
        print('-------------------------------------')
    return

def create_joint_velocity_controller(joint_index,lower_limit,upper_limit,inital_velocity):
    global robot
    joint_info = pybullet.getJointInfo(robot, joint_index) # get name of joint, to create on screen label
    joint_parameters = pybullet.addUserDebugParameter(paramName=str(joint_info[1])+'VC', rangeMin=lower_limit, rangeMax =upper_limit, startValue=inital_velocity)
    # pass the returned array to activate_position_contoller in the main loop of your script
    return [joint_index, joint_parameters]

def activate_velocity_controller(joint_parameters):
    global robot
    joint_index = joint_parameters[0]
    velocity = joint_parameters[1]
    user_velocity = pybullet.readUserDebugParameter(velocity)
    pybullet.setJointMotorControl2(robot, joint_index, pybullet.VELOCITY_CONTROL,targetVelocity= user_velocity)
    joint_info = pybullet.getJointState(robot,joint_index)
    joint_position = joint_info[0] 
    joint_velocity = joint_info[1]
    return joint_position,joint_velocity


# Simulation & Environment
pybullet.connect(pybullet.GUI) #Open pybulle simulation
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setTimeStep(1/240) #Time step

plane = pybullet.loadURDF("plane.urdf")

FLAG = pybullet.URDF_USE_SELF_COLLISION
#FLAG = (pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | pybullet.URDF_USE_SELF_COLLISION)
dir = r"C:\Users\rkobb\Desktop\2023-2\Research_Project_I\solidworks\ver12\urdf\ver12.urdf"
robot = pybullet.loadURDF(dir, [0,0,0], useFixedBase=1, flags = FLAG)

get_joint_info(robot)
pybullet.setCollisionFilterPair(robot, robot, 2, 3, 1)

#a = pybullet.addUserDebugParameter("2", -np.pi, np.pi, 0)
Joint1_VC = create_joint_velocity_controller(0,-2,2,0)
Joint2_VC = create_joint_velocity_controller(1,-2,2,0)
Joint3_VC = create_joint_velocity_controller(2,-2,2,0)
Joint4_VC = create_joint_velocity_controller(3,-2,2,0)

pybullet.changeDynamics(plane, -1, lateralFriction=0.5)
for joint_index in range(pybullet.getNumJoints(robot)): 
    pybullet.setJointMotorControl2(robot, joint_index, pybullet.POSITION_CONTROL, force = 0)
    pybullet.changeDynamics(robot, joint_index, lateralFriction=0.2376545)

pybullet.changeVisualShape(robot, 0, rgbaColor=[1, 1, 1, 1])
pybullet.changeVisualShape(robot, 1, rgbaColor=[0.85, 0.7, 1, 1])
pybullet.changeVisualShape(robot, 2, rgbaColor=[0.65, 0.5, 0.8, 1])
pybullet.changeVisualShape(robot, 3, rgbaColor=[0.45, 0.3, 0.6, 1])

pybullet.setGravity(0, 0, -9.81) # define x,y,z gravity constants

while True:
    joint1_position,joint1_velocity = activate_velocity_controller(Joint1_VC)
    joint2_position,joint2_velocity = activate_velocity_controller(Joint2_VC)
    joint3_position,joint3_velocity = activate_velocity_controller(Joint3_VC)
    joint4_position,joint4_velocity = activate_velocity_controller(Joint4_VC)
    print(joint3_position, joint4_position)
    pybullet.stepSimulation()
pybullet.disconnect()


