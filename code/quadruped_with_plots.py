import pybullet as p
import time
import pybullet_data

import numpy as np


################################################################################
# This program demonstrates basic joint movements
# It moves one leg at a time, capture location , speed and etc.
# TODO: use thread to capture robot states, speed and etc.
################################################################################

plot = True
if (plot):
  import matplotlib.pyplot as plt
#  figure=plt.figure(figsize=[10,8])
  figure, ((ax_pos,ay_pos,az_pos),(ax_orn,ay_orn,az_orn)) = plt.subplots(2,3,sharex='col',sharey='row')
  figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)
  figure.set_size_inches(10,8)


  ax_pos.set_title("Body x-location")
  ay_pos.set_title("Body y-location")
  az_pos.set_title("Body z-location")
  ax_orn.set_title("Body x-orientation")
  ay_orn.set_title("Body y-orientation")
  az_orn.set_title("Body z-orientation")

#verbose = True
verbose = False

# Parameters:
delta_t = 0.001
start_sim = 0.0
steps = 10
end_sim = delta_t * steps
t=[0 for x in range(steps)]
body_pos = [[ 0. for x in range(steps)] for y in range(3)]
body_orn = [[ 0. for x in range(steps)] for y in range(3)]

def position(jointIdx1, position1, jointIdx2, position2):
  p.setJointMotorControl2(
    bodyIndex=baseId,
    jointIndex=jointIdx1,
    controlMode=p.POSITION_CONTROL,
    targetPosition=position1,
    force=maxForce)
  p.setJointMotorControl2(
    bodyIndex=baseId,
    jointIndex=jointIdx2,
    controlMode=p.POSITION_CONTROL,
    targetPosition=position2,
    force=maxForce)

def moveLeg(joint1,joint2):
  position(joint1,0.5,joint2,0.18)
  time.sleep(stepSize)
  position(joint1,-0.5,joint2,-0.4)
  time.sleep(stepSize)
  position(joint1,0.8,joint2,0.06)
  time.sleep(stepSize)
  position(joint1,0.5,joint2,0.18)
  time.sleep(stepSize)
  
def pose0():
  moveLeg(front_left_hip_to_thigh,front_left_thigh_to_leg)
  
def pose1():
  moveLeg(rear_right_hip_to_thigh,rear_right_thigh_to_leg)

def pose2():
  moveLeg(front_right_hip_to_thigh,front_right_thigh_to_leg)

def pose3():
  moveLeg(rear_left_hip_to_thigh,rear_left_thigh_to_leg)

def robot_action(robot_state):
  switcher = {
    0: pose0,
    1: pose1,
    2: pose2,
    3: pose3
    }
  # Get the function from switcher dictionary
  func = switcher.get(robot_state)
                      #,lambda: "Invalid state")
  # Execute the function
  return func()

################################################################################
# plot
################################################################################
#plt.ion()
#img = np.random.rand(200, 320)
#img = [tandard_normal((50,100))
#image = plt.imshow(img,interpolation='none',animated=True,label="blah")
#ax = plt.gca()

################################################################################
# PyBullet
################################################################################

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setTimeStep(delta_t)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.27]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
baseId = p.loadURDF("/home/bb8/robot/git/robot_dog/urdf/quadruped.urdf",cubeStartPos, cubeStartOrientation)


mode = p.POSITION_CONTROL
numJoints = p.getNumJoints(baseId)
maxForce=1.96 # 1.96Nm = 20Kg.cm
if (verbose):
  cubePos, cubeOrn = p.getBasePositionAndOrientation(baseId)
  print(cubePos,cubeOrn)
  print( "number of joints %s" % numJoints)

jointNameToId = {}

for i in range(numJoints):
  jointInfo = p.getJointInfo(baseId,i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
#  p.setJointMotorControl2(baseId,i,controlMode=mode,force=maxForce)
  if (verbose):
    print ("Joint : ", jointInfo)

base_front_left_joint=jointNameToId['base_front_left_joint']
front_left_hip_to_thigh=jointNameToId['front_left_hip_to_thigh']
front_left_thigh_to_leg=jointNameToId['front_left_thigh_to_leg']

base_rear_left_joint=jointNameToId['base_rear_left_joint']
rear_left_hip_to_thigh=jointNameToId['rear_left_hip_to_thigh']
rear_left_thigh_to_leg=jointNameToId['rear_left_thigh_to_leg']

base_front_right_joint=jointNameToId['base_front_right_joint']
front_right_hip_to_thigh=jointNameToId['front_right_hip_to_thigh']
front_right_thigh_to_leg=jointNameToId['front_right_thigh_to_leg']

base_rear_right_joint=jointNameToId['base_rear_right_joint']
rear_right_hip_to_thigh=jointNameToId['rear_right_hip_to_thigh']
rear_right_thigh_to_leg=jointNameToId['rear_right_thigh_to_leg']

legnumbering = [
  base_front_left_joint,
  front_left_hip_to_thigh,
  front_left_thigh_to_leg,
  base_rear_left_joint,
  rear_left_hip_to_thigh,
  rear_left_thigh_to_leg,
  base_front_right_joint,
  front_right_hip_to_thigh,
  front_right_thigh_to_leg,
  base_rear_right_joint,
  rear_right_hip_to_thigh,
  rear_right_thigh_to_leg]

kp=1
kd=0.5


p.changeDynamics(baseId,-1,mass=2)

dyn = p.getDynamicsInfo(baseId,-1)
mass=dyn[0]
friction=dyn[1]
localInertiaDiagonal = dyn[2]

if (verbose):
  print("Body mass : ", mass)
  print("Friction  : ", friction)
  print("localInertiaDiagonal : ", localInertiaDiagonal)

pos1=0.5
pos2=0.17
p.resetJointState(baseId,front_left_hip_to_thigh,targetValue=pos1)
p.resetJointState(baseId,front_left_thigh_to_leg,targetValue=pos2)
p.resetJointState(baseId,front_right_hip_to_thigh,targetValue=pos1)
p.resetJointState(baseId,front_right_thigh_to_leg,targetValue=pos2)
p.resetJointState(baseId,rear_left_hip_to_thigh,targetValue=pos1)
p.resetJointState(baseId,rear_left_thigh_to_leg,targetValue=pos2)
p.resetJointState(baseId,rear_right_hip_to_thigh,targetValue=pos1)
p.resetJointState(baseId,rear_right_thigh_to_leg,targetValue=pos2)

#the fixedTimeStep and numSolverIterations are the most important parameters to trade-off quality versus performance
frequency = 8
fixedTimeStep= 1.0/frequency
stepSize=fixedTimeStep
numSolverIterations = 200
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)

#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "myQuad.mp4")


p.setRealTimeSimulation(1)
robot_state = 0

for i in range (steps):
  qKey=ord('q')
  keys=p.getKeyboardEvents()
  if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
    break
  else:
    p.stepSimulation()
    cubePos, cubeOrn = p.getBasePositionAndOrientation(baseId)
    if (verbose):
#      print(cubePos,cubeOrn)
      print(cubePos[0],cubePos[1],cubePos[2])
      
    if (plot):
      for j in range (3):
        body_pos[j][i]=cubePos[j]  # position
        body_orn[j][i]=cubeOrn[j]  # orientation
    robot_action(robot_state)
    robot_state =  (robot_state +1 ) % 4


if (plot):
  ax_pos.plot(body_pos[0], '--r', lw=1, label='x')
  ay_pos.plot(body_pos[1], '--g', lw=1, label='y')
  az_pos.plot(body_pos[2], '--b', lw=1, label='z')
  ax_orn.plot(body_orn[0], '--r', lw=1, label='x')
  ay_orn.plot(body_orn[1], '--g', lw=1, label='y')
  az_orn.plot(body_orn[2], '--b', lw=1, label='z')

plt.show()
#p.disconnect()
