# RL-Quadruped_ResearchProject_23-2


## <div align="center">2023-2 연구프로젝트I </div>
🚀 Hopping Simulation of Gear-Shifting Quadruped Robot Leg using SAC

## <div align="center">Team 서계</div>
🌟 Team Leader 이창재 (서강대학교 기계공학과 19)

🌟 Team member 강정훈 (서강대학교 기계공학과 19)

🌟 Team member 김기훈 (서강대학교 기계공학과 / 컴퓨터공학과 19)


## <div align="center">Summary</div>

🚀 1st presentation

- Customizing OpenAI Gym Cart-Pole Environment

- We try to modify terminate condition and reward function that the pole maintain 10~20 degree

- We train the cart-pole system using DQN

- Reward-Episode Graph
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/assets/97373900/8ccdf01f-ce74-4428-a3b9-0454467532eb">

- Result video of 1st week
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/blob/main/img/1_dqn_custom_cp.gif">

<br>

---

🚀 2nd Presentation

- We practice to use Mlp stable_baseline3 Soft-Actor Critic to train robot appropriately
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/blob/main/img/2_2d_pusher.gif">

- Our 1st modeling + URDF Extraction form Solidworks → Self collision problem occur
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/blob/main/img/2_1st_model.gif">

- We decide to make new hopping leg model without any prismatic link 


<br>

---

🚀 3rd Presentation

- Our 2nd modeling
  <br>
  <img width="600" src="https://github.com/cobang0111/DRL_2023_Fall/blob/main/3_solidworks_model_v1.png?raw=true">


- First SAC Try (Reward = Difference of joint 1 angle for each step, total_episode = 3000)
  <br>
  <img width="600" src="https://github.com/cobang0111/DRL_2023_Fall/blob/main/3_SAC_231108_1.gif?raw=true">

- We got feedback that the rotational axis can affect to hopper 

 <br>

---

🚀 4th Presentation

- Our 3rd modeling
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/assets/97373900/9ccc3ed3-1b02-4ee7-ac25-7190c05797c3">
 
- To leverage the advantages of simulation, an experiment was conducted by setting a point mass of 10kg at the topmost link of a quadruped robot's leg

- Initial Setting - Upper leg : 1kg / 500mm / -30˚ | Lower leg : 1kg / 500mm / 60˚

- Result
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/blob/main/img/4_final_training.GIF">

 <br>

---

🚀 5th Presentation

- Ultimately, by training the hopping motion of a quadruped robot's legs in a simulation, we acquired data on the relationship between motor power, torque, and velocity during the execution of this action.
- Ideal Torque-Velocity Graph
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/assets/97373900/c9aad51a-191c-4487-a229-72c570629359">
- We set the P_MAX as 10W
- Torque-Velocity Graph from Simulation
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/assets/97373900/15aa507a-7314-4c04-a092-ec69faa1677d">
- Height-Timestep Graph from Simulation
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/assets/97373900/56d8a654-65f8-4632-b614-0b62a5f89b0b">
- Trained Hopping Motion
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/blob/main/img/5_hop.gif">
- We obtained two velocity control factors as network outputs from the SAC network, and when applied, we created an effect of gear-ratio shifting by making the torque act as the value obtained by dividing maximum power by velocity.
- By examining the actual graph, we could observe points plotted along the maximum power line, confirming that the network was trained to generate optimal torque and velocity necessary for the hopping action.
  

- Additionally, we try to train Boston Dynamics SPOT using SAC but it failed
  <br>
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/blob/main/img/5_SPOT.gif">
  
- For the locomotion of quadruped robots, it appears necessary to employ more accurate reward functions, set end conditions, and set constraints

			  


