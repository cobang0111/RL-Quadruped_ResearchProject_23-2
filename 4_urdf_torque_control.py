import pybullet
import pybullet_data

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

def create_joint_torque_controller(joint_index,lower_limit,upper_limit,inital_torque):
    global robot
    joint_info = pybullet.getJointInfo(robot, joint_index) # get name of joint, to create on screen label
    joint_parameters = pybullet.addUserDebugParameter(paramName=str(joint_info[1])+'VC', rangeMin=lower_limit, rangeMax =upper_limit, startValue=inital_torque)
    # pass the returned array to activate_position_contoller in the main loop of your script
    return [joint_index, joint_parameters]

def activate_torque_controller(joint_parameters):
    global robot
    joint_index = joint_parameters[0]
    torque = joint_parameters[1]
    user_torque = pybullet.readUserDebugParameter(torque)
    pybullet.setJointMotorControl2(robot, joint_index, pybullet.TORQUE_CONTROL,force = user_torque)
    joint_info = pybullet.getJointState(robot,joint_index)
    joint_position = joint_info[0] 
    joint_torque = joint_info[1]
    return joint_position,joint_torque


# Simulation & Environment
pybullet.connect(pybullet.GUI) #Open pybulle simulation
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setTimeStep(1/400) #Time step

plane = pybullet.loadURDF("plane.urdf")

#FLAG = pybullet.URDF_USE_SELF_COLLISION
FLAG = (pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | pybullet.URDF_USE_SELF_COLLISION)
#dir_frame = r"C:\Users\rkobb\Desktop\2023-2\Research_Project_I\solidworks\111605_00\urdf\111605_00.urdf"
#dir_robot = r"C:\Users\rkobb\Desktop\2023-2\Research_Project_I\solidworks\111604\urdf\111604.urdf"
dir_robot = r"C:\Users\rkobb\Desktop\2023-2\Research_Project_I\solidworks\112101_3\urdf\112101_3.urdf"
#frame = pybullet.loadURDF(dir_frame, [0,0,0], useFixedBase=1)
robot = pybullet.loadURDF(dir_robot, [0,0,1.2], useFixedBase=1)

get_joint_info(robot)

Joint1_VC = create_joint_torque_controller(0,-10,10,0)
Joint2_VC = create_joint_torque_controller(1,-10,10,0)

pybullet.changeDynamics(plane, -1, lateralFriction=0.5)
for joint_index in range(pybullet.getNumJoints(robot)): 
    pybullet.setJointMotorControl2(robot, joint_index, pybullet.VELOCITY_CONTROL, force = 0)
    pybullet.changeDynamics(robot, joint_index, lateralFriction=0.2376545)

#pybullet.changeVisualShape(robot, 0, rgbaColor=[1, 1, 1, 1])
pybullet.changeVisualShape(robot, 0, rgbaColor=[0.85, 0.7, 1, 1])
pybullet.changeVisualShape(robot, 1, rgbaColor=[0.65, 0.5, 0.8, 1])
pybullet.changeVisualShape(robot, 2, rgbaColor=[0.45, 0.3, 0.6, 1])

pybullet.setGravity(0, 0, 0) # define x,y,z gravity constants

while True:
    joint1_position,joint1_velocity = activate_torque_controller(Joint1_VC)
    joint2_position,joint2_velocity = activate_torque_controller(Joint2_VC)

    #print(joint3_position)
    pybullet.stepSimulation()
pybullet.disconnect()


