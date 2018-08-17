import pybullet as p
import time
import pybullet_data

################################################################################
# This program demonstrates basic joint movements
# It moves one leg at a time
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


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.27]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
baseId = p.loadURDF("/home/bb8/robot/git/robot_dog/urdf/quadruped.urdf",cubeStartPos, cubeStartOrientation)


cubePos, cubeOrn = p.getBasePositionAndOrientation(baseId)
print(cubePos,cubeOrn)


mode = p.POSITION_CONTROL
numJoints = p.getNumJoints(baseId)
maxForce=1.96 # 1.96Nm = 20Kg.cm
print( "number of joints %s" % numJoints)

jointNameToId = {}

for i in range(numJoints):
  jointInfo = p.getJointInfo(baseId,i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
#  p.setJointMotorControl2(baseId,i,controlMode=mode,force=maxForce)
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

p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "myQuad.mp4")


p.setRealTimeSimulation(1)
robot_state = 0
for i in range (100):
  qKey=ord('q')
  keys=p.getKeyboardEvents()
  if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
    break
  else:
    cubePos, cubeOrn = p.getBasePositionAndOrientation(baseId)
    print(cubePos,cubeOrn)
    robot_action(robot_state)
    robot_state =  (robot_state +1 ) % 4
    time.sleep(stepSize)
  
p.disconnect()
