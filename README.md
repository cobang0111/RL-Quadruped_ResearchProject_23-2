# DRL_2023_Fall



## <div align="center">2023-2 연구프로젝트I </div>
🚀 PyTorch Soft Actor Critic 을 이용한 Pybullet 사족보행로봇 다리 Hopping 최적화

## <div align="center">Team 서계</div>
🌟 Team Leader 이창재 (서강대학교 기계공학과 19)

🌟 Team member 강정훈 (서강대학교 기계공학과 19)

🌟 Team member 김기훈 (서강대학교 기계공학과 / 컴퓨터공학과 19)

## <div align="center">Video</div>

🚀 See our operation video on the YouTube (In progress)

[Youtube_link]()


## <div align="center">Summary</div>


🚀 Our Record


- First Solidworks Model

<p align="center"><img width="800" src="g"></p>


- First SAC Try (Reward = Difference of joint 1 angle for each step, time_steps = 3000) 

<p align="center"><img width="800" src="https://github.com/cobang0111/DRL_2023_Fall/blob/main/3_SAC_231108_1.gif?raw=true"></p>
 
- DRL our model by PyTorch SAC

<p align="center">First Code</p>

```python
from stable_baselines3.sac.policies import MlpPolicy
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.results_plotter import load_results, ts2xy
log_dir = "/tmp/gym/"
os.makedirs(log_dir, exist_ok=True)

# Simulation & Environment
pybullet.connect(pybullet.DIRECT) #Open directly without GUI
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

pybullet.setGravity(0, 0, -9.81) # define x,y,z gravity constants
pybullet.setTimeStep(1/240) #Time step

FLAG = pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT | pybullet.URDF_USE_SELF_COLLISION

plane = pybullet.loadURDF("plane.urdf")
robot = pybullet.loadURDF("/content/ver8/urdf/ver8.urdf",[0,0,0],useFixedBase=1, flags = FLAG)

# camera parameters
cam_target_pos = [0, -0.2, 0.2]
cam_distance = 2.5
cam_yaw, cam_pitch, cam_roll = 60, 0, 0
cam_width, cam_height = 480, 360
cam_up, cam_up_axis_idx, cam_near_plane, cam_far_plane, cam_fov = [0, 0, 1], 2, 0.01, 100, 60


class minicatEnv(gym.Env):
    def __init__(self, initial_position):
        super(minicatEnv, self).__init__()
        self.action_space = spaces.Box(low=-1, high=1, shape=(2, ))
        
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(2, 4))
        self.plane = pybullet.loadURDF("plane.urdf")
        self.robot = pybullet.loadURDF("/content/ver8/urdf/ver8.urdf",[0,0,0],useFixedBase=1, flags = FLAG)
        self.previous_position = initial_position

    def position_update(self, new_position):
      difference = new_position - self.previous_position
      self.previous_position = new_position
      return difference

    def step(self, action):
        # Apply torque to the joint
        pybullet.setGravity(0, 0, -9.81)
        position2 = action[0]
        position3 = action[1]
        pybullet.setJointMotorControl2(self.robot, 0, pybullet.POSITION_CONTROL, force = 0)
        pybullet.setJointMotorControl2(self.robot, 1, pybullet.POSITION_CONTROL, force = 0)
        pybullet.setJointMotorControl2(self.robot, 2, pybullet.POSITION_CONTROL, targetPosition=position2, force = 5)
        pybullet.setJointMotorControl2(self.robot, 3, pybullet.POSITION_CONTROL, targetPosition=position3, force = 5)

        pybullet.changeDynamics(plane, -1, lateralFriction=0.5)
        #for joint_index in range(pybullet.getNumJoints(robot)): 
            #pybullet.setJointMotorControl2(self.robot, joint_index, pybullet.POSITION_CONTROL, force = 0)
            #pybullet.changeDynamics(self.robot, joint_index, lateralFriction=0.2376545)
        pybullet.changeDynamics(self.robot, -1, lateralFriction=0.5)
        pybullet.changeDynamics(self.robot, 3, lateralFriction=0.5)
        

        pybullet.changeVisualShape(self.robot, 0, rgbaColor=[1, 1, 1, 1])
        pybullet.changeVisualShape(self.robot, 1, rgbaColor=[0.85, 0.7, 1, 1])
        pybullet.changeVisualShape(self.robot, 2, rgbaColor=[0.65, 0.5, 0.8, 1])
        pybullet.changeVisualShape(self.robot, 3, rgbaColor=[0.45, 0.3, 0.6, 1])
        
        # Step the simulation
        pybullet.stepSimulation()

        # Get the joint's angle and velocity
        joint_info_0 = pybullet.getJointState(self.robot, 0)
        joint_info_1 = pybullet.getJointState(self.robot, 1)
        joint_info_2 = pybullet.getJointState(self.robot, 2)
        joint_info_3 = pybullet.getJointState(self.robot, 3)

        joint_angle_0 = joint_info_0[0]
        joint_velocity_0 = joint_info_0[1]

        joint_angle_1 = joint_info_1[0]
        joint_velocity_1 = joint_info_1[1]

        joint_angle_2 = joint_info_2[0]
        joint_velocity_2 = joint_info_2[1]

        joint_angle_3 = joint_info_3[0]
        joint_velocity_3 = joint_info_3[1]

        pos_diff_0 = self.position_update(joint_angle_0)
        pos_diff_1 = self.position_update(joint_angle_1)
         
        # Calculate reward
        reward = 1 * abs(pos_diff_1)

        # The episode is done if the pole is more than some degrees from vertical
        done = joint_angle_1 > np.radians(90)

        arr = [[joint_angle_0, joint_angle_1, joint_angle_2, joint_angle_3], [joint_velocity_0, joint_velocity_1, joint_velocity_2, joint_velocity_3]]

        return np.array(arr), reward, done, {}

    def reset(self):
        pybullet.resetSimulation()
        self.plane = pybullet.loadURDF("plane.urdf")
        self.robot = pybullet.loadURDF("/content/ver8/urdf/ver8.urdf",[0,0,0],useFixedBase=1, flags = FLAG)
        pybullet.setGravity(0, 0, -9.81)
        initial_angle_0 = 0
        initial_angle_1 = np.radians(-20)
        initial_angle_2 = np.random.uniform(np.radians(-179), np.radians(179))
        initial_angle_3 = np.random.uniform(np.radians(-179), np.radians(179))
        
        pybullet.resetJointState(self.robot, 0, initial_angle_0)  # Set the initial angle of the joint
        pybullet.resetJointState(self.robot, 1, initial_angle_1)
        pybullet.resetJointState(self.robot, 2, initial_angle_2) 
        pybullet.resetJointState(self.robot, 3, initial_angle_3)

        joint_info_0 = pybullet.getJointState(self.robot, 0)  # Get the initial joint state
        joint_info_1 = pybullet.getJointState(self.robot, 1)
        joint_info_2 = pybullet.getJointState(self.robot, 2)
        joint_info_3 = pybullet.getJointState(self.robot, 3)

        
        joint_angle_0 = joint_info_0[0]
        joint_velocity_0 = joint_info_0[1]

        joint_angle_1 = joint_info_1[0]
        joint_velocity_1 = joint_info_1[1]

        joint_angle_2 = joint_info_2[0]
        joint_velocity_2 = joint_info_2[1]

        joint_angle_3 = joint_info_3[0]
        joint_velocity_3 = joint_info_3[1]

        arr = [[joint_angle_0, joint_angle_1, joint_angle_2, joint_angle_3], [joint_velocity_0, joint_velocity_1, joint_velocity_2, joint_velocity_3]]

        return np.array(arr)

    def render(self):
        cam_view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(cam_target_pos, cam_distance, cam_yaw, cam_pitch, cam_roll, cam_up_axis_idx)
        cam_projection_matrix = pybullet.computeProjectionMatrixFOV(cam_fov, cam_width*1./cam_height, cam_near_plane, cam_far_plane)
        image = pybullet.getCameraImage(cam_width, cam_height, cam_view_matrix, cam_projection_matrix)[2][:, :, :3]
        image = np.ascontiguousarray(image)
        return image

    def close(self):
        pybullet.disconnect()


def plot_results(log_folder, title="Learning_Curve"):
  x, y = ts2xy(load_results(log_folder), 'timesteps')
  plt.plot(x,y)
  plt.title(title)
  plt.xlabel('Numbers of Timesteps')
  plt.ylabel('Rewards')
  plt.show()



# Create the environment
env = minicatEnv(initial_position = 0)

model = SAC('MlpPolicy', env, verbose=1)
model.learn(total_timesteps=10000)
#plot_results(log_dir+'monitor.csv')
model.save("sac_minicat_env")

del model # remove to demonstrate saving and loading

model = SAC.load("sac_minicat_env")

vid = imageio_ffmpeg.write_frames('vid.mp4', (cam_width, cam_height), fps=30)
vid.send(None) # seed the video writer with a blank frame


# Enjoy trained agent
for i in range(1):
    obs = env.reset()
    for j in range(500):
        action, _states = model.predict(obs)
        #print(action)
        obs, rewards, done, info = env.step(action)
        image = env.render()
        vid.send(image)
        if done:
            break

vid.close()
env.close()

```


			  

## <div align="center">Reference</div>



