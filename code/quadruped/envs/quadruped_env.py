################################################################################
# Copyright (c) 2018 Chi K. Lee
# Release under BSD 3-Clause License 
################################################################################
''' 
    In this program, the robot tries to keep the body at the height of 0.275 and
    as steady as possible.
'''
import os
import math
import numpy as np
import re

import gym
from gym import spaces
from gym.utils import seeding

import pybullet as p
import pybullet_data

class QuadrupedEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }


    def __init__(self, render=False):
        self._observation = []
        self._episode_count =0
        self.action_space = spaces.Discrete(144) # 12 joints, each with 12 angles
        # 24 parameters to observe
        # pos , 3 
        # orn , 3
        # linear, 3
        # angular, 3
        # joints , 12
        inf=999
        self.observation_space = spaces.Box(np.array([-inf]*24),np.array([inf]*24))
        if (render):
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        self._seed()
        p.resetSimulation()
        self._maxJointForce=1.96 # 1.96Nm = 20Kg.cm
        cameraDistance=1.2
        cameraPitch=-35.4
        cameraYaw=26.8
#        cameraTargetPosition=[0.4,0.2,-0.2]
        cameraTargetPosition=[0.1,0.07,-0.16]
        p.resetDebugVisualizerCamera(cameraDistance,cameraYaw,cameraPitch,cameraTargetPosition)
        p.resetSimulation()
        p.setGravity(0,0,-9.8) # m/s^2
        p.setTimeStep(0.01) # sec
        planeId = p.loadURDF("plane.urdf")
        path = os.path.abspath(os.path.dirname(__file__))
        
        cubeStartPos = [0,0,0.28]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        self.botId = p.loadURDF("/home/bb8/robot/git/robot_dog/urdf/quadruped.urdf",                          
                                cubeStartPos,
                                cubeStartOrientation)
        self._stateId =p.saveState()
        
    def _create_joint_map(self):
        numJoints = p.getNumJoints(self.botId)
        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.botId,i)
            jointName=jointInfo[1].decode('UTF-8')
        
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def step(self, action):
        self._move_legs(action)
        p.stepSimulation()
        self._observation = self._compute_observation()
        reward = self._compute_reward()
        done = self._compute_done()
        
        self._envStepCounter += 1
        
        return np.array(self._observation), reward, done, {}
        
    def reset(self):
        # reset is called once at initialization of simulation
        self._episode_count +=1        


        p.restoreState(self._stateId)
        
        # capture video
        if (True):
            if (self._episode_count % 10 ==1):

                output_file="episode_%d.mp4" % self._episode_count
                print("Saving video : ",output_file)
                self._state_log = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,output_file)
        self._envStepCounter = 0

        

        # you *have* to compute and return the observation from reset()
        self._observation = self._compute_observation()
        return np.array(self._observation)

    def _move_legs(self, action):
        numJoints = 12
        numAngles = 12
        joint_number = int(action/numAngles)
        joint_position = action % numJoints
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=joint_number,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self._get_joint_angle(joint_number,joint_position,numAngles),
                                force=self._maxJointForce)
        
    def _get_joint_angle(self,joint_number,pos,max_pos):
        jointInfo = p.getJointInfo(self.botId,joint_number)
        jointName=jointInfo[1].decode('UTF-8')
        if 'hip_to_thigh' in jointName:
            minAngle=0
            maxAngle=math.pi/2
        else:
            minAngle=-math.pi/4
            maxAngle=math.pi/4
        return (maxAngle-minAngle)*pos/(max_pos-1)+minAngle
    
        
    def _compute_observation(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
#        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        obs =  [cubePos[0],cubePos[1],cubePos[2],
                cubeOrn[0],cubeOrn[1],cubeOrn[2],
                linear[0],linear[1],linear[2],
                angular[0],angular[1],angular[2]]
        numJoints=p.getNumJoints(self.botId)
        for i in range(numJoints):
            jointPosition,_,_,_=p.getJointState(self.botId,i)
            obs.append(jointPosition)
        return obs
               
               
    def _compute_reward(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        maxScore=0.15
        targetZ=0.275
        panelty=0.1*(abs(cubeOrn[0])+abs(cubeOrn[1])+abs(cubeOrn[2]))
        if abs(cubeOrn[0])>0.5 or abs(cubeOrn[1])>0.5 or abs(cubeOrn[2])>0.5 or cubePos[2]<0.15:
            panelty=20
        return (maxScore-abs(cubePos[2]-targetZ)-panelty)

    def _compute_done(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        if (cubePos[2] < 0.15 or self._envStepCounter >= 1500 or abs(cubeOrn[0])>0.5 or abs(cubeOrn[1])>0.5 or abs(cubeOrn[2])>0.5):
            if (self._episode_count % 10 == 1):
                p.stopStateLogging(self._state_log)
            return True
        else:
            return False


    def _render(self, mode='human', close=False):
        pass

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
