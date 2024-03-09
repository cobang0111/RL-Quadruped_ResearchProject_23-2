# RL-Quadruped_ResearchProject_23-2


## <div align="center">2023-2 연구프로젝트I </div>
🚀 Optimizing Simulation of Hopping Action of Gear-Shifting Quadruped Robot Leg using SAC

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
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/assets/97373900/8ccdf01f-ce74-4428-a3b9-0454467532eb">

- Result video of 1st week
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/assets/97373900/dcee41ee-5394-416f-8a70-bc528a846b8d">

<br>

---

🚀 2nd Presentation

- We practice to use Mlp stable_baseline3 Soft-Actor Critic to train robot appropriately
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/assets/97373900/d824fadb-cc62-407a-9908-82510352b4f2">

- Our 1st modeling + URDF Extraction form Solidworks → Self collision problem occur
  <img width="600" src="https://github.com/cobang0111/RL-Quadruped_ResearchProject_23-2/assets/97373900/246bf0ee-b5f5-4411-9ac8-269ccec4a578">

- We decide to make new hopping leg model without any prismatic link 


<br>

---

🚀 3rd Presentation

- Our 2nd modeling
  <img width="600" src="https://github.com/cobang0111/DRL_2023_Fall/blob/main/3_solidworks_model_v1.png?raw=true">


- First SAC Try (Reward = Difference of joint 1 angle for each step, total_episode = 3000)
- <img width="600" src="https://github.com/cobang0111/DRL_2023_Fall/blob/main/3_SAC_231108_1.gif?raw=true">

 <br>

---

🚀 4th Presentation

- Our 3rd modeling - we 



			  

## <div align="center">Reference</div>



