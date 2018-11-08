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


    def __init__(self, render=True):
        self._observation = []
        self._jointIdMap = [0] *8
        self._episode_count =0
        self.action_space = spaces.Discrete(96)
        self.observation_space = spaces.Box(np.array([-math.pi, -math.pi, -5]), 
                                            np.array([math.pi, math.pi, 5])) # pitch, gyro, com.sp.
        if (render):
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version

        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        self._seed()

    def _create_joint_map(self):
        numJoints = p.getNumJoints(self.botId)
        jointNameToId = {}
        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.botId,i)
            jointName=jointInfo[1].decode('UTF-8')
            jointNameToId[jointName] = jointInfo[0]
#            print("Joint %d = (%s)" % (i,jointName))
            if (re.search('front_left_',jointName)):
                legId=0
            elif (re.search('front_right_',jointName)):
                legId=1
            elif (re.search('rear_left_',jointName)):
                legId=2
            elif (re.search('rear_right_',jointName)):
                legId=3
            else:
#                print("Warning: ignore jointName ( %s )" % jointName)
                legId=-1
            if (re.search('thigh_to_leg',jointName)):
                isLegJoint=1
            elif (re.search('hip_to_thigh',jointName)):                
                isLegJoint=0
            else:
                isLegJoint=-1
            if ((legId>=0) and (isLegJoint>=0)):
                jid=legId*2+isLegJoint
                self._jointIdMap[jid]=i
#                print("Mapping %d to %d (%s)" %(jid,i,jointName))

#        if (verbose):
#            print ("Joint : ", jointInfo)
        
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
#        print("Episode %d" % self._episode_count)

        # capture video
        if (True):
            if (self._episode_count % 10 ==1):
                output_file="episode_%d.mp4" % self._episode_count
                self._state_log = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,output_file)
            
        self.vt = 0
        self.vd = 0
        self.maxV = 24.6 # 235RPM = 24,609142453 rad/sec
        self._envStepCounter = 0
        self._maxJointForce=1.96 # 1.96Nm = 20Kg.cm
        cameraDistance=2.0
        cameraPitch=-35.4
        cameraYaw=26.8
        cameraTargetPosition=[0.4,0.2,-0.2]
        p.resetDebugVisualizerCamera(cameraDistance,cameraYaw,cameraPitch,cameraTargetPosition)
        p.resetSimulation()
        p.setGravity(0,0,-9.8) # m/s^2
        p.setTimeStep(0.01) # sec
        planeId = p.loadURDF("plane.urdf")
        cubeStartPos = [0,0,0.28]
        cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
        path = os.path.abspath(os.path.dirname(__file__))
        self.botId = p.loadURDF("/home/bb8/robot/git/robot_dog/urdf/quadruped.urdf",                          
                                cubeStartPos,
                                cubeStartOrientation)
        self._create_joint_map()

        # you *have* to compute and return the observation from reset()
        self._observation = self._compute_observation()
        return np.array(self._observation)

    def _move_legs(self, action):
        # 8 joints , each with 12 angles
        # total 96 actions (0-95)
        # range
        # hip_to_thigh : 0 to pi/2
        # thing_to_leg : -pi/4 to pi/4
#        print("Action = {}".format(action))
        numJoints = 8
        numAngles = 12
        joint_number = int(action/numAngles)
        joint_position = action % numJoints
        leg_number = joint_number % 4
        isLeg = int(joint_number/4)
#        print("Leg {}, isLeg= {}, angle = {}".format(leg_number,isLeg,joint_position))
        jid=leg_number*2+isLeg
#        print("--------------------------------------\n")
        p.setJointMotorControl2(bodyUniqueId=self.botId, 
                                jointIndex=self._jointIdMap[jid], 
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=self._get_joint_angle(isLeg,joint_position,numAngles),
                                force=self._maxJointForce)

    def _get_joint_angle(self,isLeg,pos,max_pos):
        if (isLeg==1):
            minAngle=-math.pi/4
            maxAngle=math.pi/4
        else:
            minAngle=0
            maxAngle=math.pi/2
        return (maxAngle-minAngle)*pos/(max_pos-1)+minAngle
        
    def _compute_observation(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        cubeEuler = p.getEulerFromQuaternion(cubeOrn)
        linear, angular = p.getBaseVelocity(self.botId)
        #TODO: observe joint angles
        return [cubeEuler[0],angular[0],self.vt]

    def _compute_reward(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
        maxScore=0.15
        targetZ=0.25
        return (maxScore-abs(cubePos[2]-targetZ))

    def _compute_done(self):
        cubePos, cubeOrn = p.getBasePositionAndOrientation(self.botId)
#        print("%0.3f, %0.3f" % (cubeOrn[0],cubeOrn[1]))
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
