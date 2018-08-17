import pybullet as p
import time
import pybullet_data

################################################################################
# This program demonstrates basic joint movements
# All it does is to apply same position control for all 4 legs to move the
# robot body up and down repeatedly.
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
  


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.27]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
baseId = p.loadURDF("urdf/quadruped.urdf",cubeStartPos, cubeStartOrientation)


cubePos, cubeOrn = p.getBasePositionAndOrientation(baseId)
print(cubePos,cubeOrn)

#the fixedTimeStep and numSolverIterations are the most important parameters to trade-off quality versus performance
frequency = 10.0
fixedTimeStep= 1.0/frequency
numSolverIterations = 500
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)

mode = p.POSITION_CONTROL
numJoints = p.getNumJoints(baseId)
maxForce=1.96 # 1.96Nm = 20Kg.cm
#maxForce=20 # arbitary
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

#p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT,
p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "myQuad.mp4")

robotstate = 0
p.setRealTimeSimulation(1)
for i in range (200):
  if (robotstate ==0):
    pos1=0.5
    pos2=0.17
    position(front_left_hip_to_thigh,pos1,front_left_thigh_to_leg,pos2)
    position(front_right_hip_to_thigh,pos1,front_right_thigh_to_leg,pos2)
    position(rear_left_hip_to_thigh,pos1,rear_left_thigh_to_leg,pos2)
    position(rear_right_hip_to_thigh,pos1,rear_right_thigh_to_leg,pos2)
  else:
    pos1=1.0
    pos2=-0.5
    position(front_left_hip_to_thigh,pos1,front_left_thigh_to_leg,pos2)
    position(front_right_hip_to_thigh,pos1,front_right_thigh_to_leg,pos2)
    position(rear_left_hip_to_thigh,pos1,rear_left_thigh_to_leg,pos2)
    position(rear_right_hip_to_thigh,pos1,rear_right_thigh_to_leg,pos2)
  robotstate = 1 - robotstate    
  time.sleep(fixedTimeStep)



cubePos, cubeOrn = p.getBasePositionAndOrientation(baseId)
print(cubePos,cubeOrn)
  
p.disconnect()
