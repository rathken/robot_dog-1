################################################################################
# Copyright (c) 2018 Chi K. Lee
# Release under BSD 3-Clause License 
################################################################################
import pybullet as p
import time
import pybullet_data
import numpy as np

################################################################################
# This program demonstrates basic joint movements
# It moves one leg at a time, capture location , speed and etc.
# TODO: use thread to capture robot states, speed and etc.
################################################################################

################################################################################
# global parameters
################################################################################
plot = True
if (plot):
  import matplotlib.pyplot as plt


#verbose = True
verbose = False

useFriction=False

# Parameters:
delta_t = 0.001
start_sim = 0.0
steps = 50
end_sim = delta_t * steps

  
################################################################################
# functions
################################################################################

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
#-------------------------------------------------------------------------------
def moveLeg(joint1,joint2):
  position(joint1,0.5,joint2,0.18)
  time.sleep(stepSize)
  position(joint1,-0.5,joint2,-0.4)
  time.sleep(stepSize)
  position(joint1,0.8,joint2,0.06)
  time.sleep(stepSize)
  position(joint1,0.5,joint2,0.18)
  time.sleep(stepSize)
#-------------------------------------------------------------------------------  
def pose0():
  moveLeg(front_left_hip_to_thigh,front_left_thigh_to_leg)
#-------------------------------------------------------------------------------  
def pose1():
  moveLeg(rear_right_hip_to_thigh,rear_right_thigh_to_leg)
#-------------------------------------------------------------------------------
def pose2():
  moveLeg(front_right_hip_to_thigh,front_right_thigh_to_leg)
#-------------------------------------------------------------------------------
def pose3():
  moveLeg(rear_left_hip_to_thigh,rear_left_thigh_to_leg)
#-------------------------------------------------------------------------------
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
#-------------------------------------------------------------------------------
def plot_graphs(ax_pos, ax_orn, ax_vel, ax_avel):
  for i in range(3):
    ax_pos[i].set_ylim(auto=True)
    ax_orn[i].set_ylim(auto=True)
    ax_vel[i].set_ylim(auto=True)
    ax_avel[i].set_ylim(auto=True)
    ax_pos[i].plot(body_pos[i], plot_colors[i], lw=1, label=plot_labels[i])
    ax_orn[i].plot(body_orn[i], plot_colors[i], lw=1, label=plot_labels[i])
    ax_vel[i].plot(body_vel[i], plot_colors[i], lw=1, label=plot_labels[i])
    ax_avel[i].plot(body_avel[i], plot_colors[i], lw=1, label=plot_labels[i])  
#-------------------------------------------------------------------------------
def create_figure(figure, ax_pos, ax_orn, ax_vel, ax_avel):
  figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)
  figure.set_size_inches(10,8)
  ax_pos[0].set_title("Body x-location")
  ax_pos[1].set_title("Body y-location")
  ax_pos[2].set_title("Body z-location")
  ax_orn[0].set_title("Body x-orientation")
  ax_orn[1].set_title("Body y-orientation")
  ax_orn[2].set_title("Body z-orientation")
  ax_vel[0].set_title("Body x-velocity")
  ax_vel[1].set_title("Body y-velocity")
  ax_vel[2].set_title("Body z-velocity")
  ax_avel[0].set_title("Body x-angular velocity")
  ax_avel[1].set_title("Body y-angular velocity")
  ax_avel[2].set_title("Body z-angular velocity")

#-------------------------------------------------------------------------------
def read_robot_status(i, body_pos, body_orn, body_vel, body_avel):
  pos, orn = p.getBasePositionAndOrientation(baseId)
  vel, avel = p.getBaseVelocity(baseId)
  for j in range(3):
    body_pos[j][i]=pos[j]
    body_orn[j][i]=orn[j]
    body_vel[j][i]=vel[j]
    body_avel[j][i]=avel[j]
  
################################################################################
# PyBullet
################################################################################

if (plot):
  figure, (ax_pos,ax_orn,ax_vel,ax_avel) = plt.subplots(4,3)
  create_figure(figure, ax_pos, ax_orn, ax_vel, ax_avel)

t=[0 for x in range(steps)]
body_pos = [[ 0. for x in range(steps)] for y in range(3)]
body_orn = [[ 0. for x in range(steps)] for y in range(3)]
body_vel = [[ 0. for x in range(steps)] for y in range(3)]
body_avel = [[ 0. for x in range(steps)] for y in range(3)]
plot_colors = ['--r','--g','--b']
plot_labels = ['x','y','z']

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
cameraDistance=2.0
cameraPitch=-35.4
cameraYaw=26.8
cameraTargetPosition=[0.4,0.2,-0.2]
p.resetDebugVisualizerCamera(cameraDistance,cameraYaw,cameraPitch,cameraTargetPosition)
p.setTimeStep(delta_t)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
planeId = p.loadURDF("plane.urdf")
#cubeStartPos = [0,0,0.27]
cubeStartPos = [0,0,0.28]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
baseId = p.loadURDF("/home/bb8/robot/git/robot_dog/urdf/quadruped.urdf",cubeStartPos, cubeStartOrientation)


mode = p.POSITION_CONTROL
numJoints = p.getNumJoints(baseId)
maxForce=1.96 # 1.96Nm = 20Kg.cm
if (verbose):
  basePos, baseOrn = p.getBasePositionAndOrientation(baseId)
  print(basePos,baseOrn)
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


#rollFriction=3
#p.changeDynamics(baseId,front_left_thigh_to_leg,rollingFriction=rollFriction)
#p.changeDynamics(baseId,front_right_thigh_to_leg,rollingFriction=rollFriction)
#p.changeDynamics(baseId,rear_left_thigh_to_leg,rollingFriction=rollFriction)
#p.changeDynamics(baseId,rear_right_thigh_to_leg,rollingFriction=rollFriction)


if (useFriction):
  latFriction=0.6
  p.changeDynamics(baseId,front_left_thigh_to_leg,lateralFriction=latFriction)
  p.changeDynamics(baseId,front_right_thigh_to_leg,lateralFriction=latFriction)
  p.changeDynamics(baseId,rear_left_thigh_to_leg,lateralFriction=latFriction)
  p.changeDynamics(baseId,rear_right_thigh_to_leg,lateralFriction=latFriction)
  p.changeDynamics(planeId,-1,lateralFriction=latFriction)
  legD = p.getDynamicsInfo(baseId,front_left_thigh_to_leg)
  legFriction=legD[1]
  #print("Leg friction = {}".format(legFriction))

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
#p.changeDynamics(
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
frequency = 20
fixedTimeStep= 1.0/frequency
stepSize=fixedTimeStep
numSolverIterations = 200
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
#p.setPhysicsEngineParameter(fixedTimeStep=fixedTimeStep, numSolverIterations=numSolverIterations, numSubSteps=2)

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
    if (plot):
      read_robot_status(i, body_pos, body_orn, body_vel, body_avel)
        
    robot_action(robot_state)
    robot_state =  (robot_state +1 ) % 4


if (plot):
  plot_graphs(ax_pos, ax_orn, ax_vel, ax_avel)

plt.show()
#p.disconnect()
