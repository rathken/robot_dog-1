import pybullet as p
import time
import math
import pybullet_data
physicsClient = p.connect(p.GUI)
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0,0)



####################################################
# Lower Leg
####################################################
lowerLegRadius=0.025/2
lowerLegLength=0.14
lowerLegMass=0.03

colLowerLegID=p.createCollisionShape(p.GEOM_CYLINDER, height=lowerLegLength, radius=lowerLegRadius)
visualLowerLegID = -1

basePosition = [0,0,0]

roll=0*math.pi/180
pitch=0*math.pi/180
yaw=0*math.pi/180

baseOrientation = p.getQuaternionFromEuler([roll,pitch,yaw])

#lowerLegID = p.createMultiBody(lowerLegMass,colLowerLegID,visualLowerLegID,basePosition,baseOrientation)

####################################################
# Thigh
####################################################
thighLength=0.15
thighThickness=0.006
thighWidth=0.035
thighMass=0.02
colThighID=p.createCollisionShape(p.GEOM_BOX, halfExtents=[thighLength/2,thighWidth/2,thighThickness/2])

####################################################
# Hip
####################################################
hipLength=0.045
hipRadius=lowerLegRadius
hipMass=0.02
colHipID=p.createCollisionShape(p.GEOM_CYLINDER,radius=hipRadius,height=hipLength)

####################################################
# Links
####################################################
link_Masses=[thighMass,hipMass]
linkCollisionShapeIndices=[colThighID,colHipID]
linkVisualShapeIndices=[-1,-1]
linkPositions=[
    [thighLength/2-0.015,0,lowerLegLength/2+0.015],
    [thighLength/2-0.015,0,thighThickness+0.015*2]
]
linkOrientations=[[0,0,0,1],[0,0,0,1]]
linkInertialFramePositions=[[0,0,0],[0,0,0]]
linkInertialFrameOrientations=[[0,0,0,1],[0,0,0,1]]
indices=[0,1]
jointTypes=[p.JOINT_REVOLUTE,p.JOINT_REVOLUTE]
axis=[[0,1,0],[0,1,0]]

legID = p.createMultiBody(lowerLegMass,colLowerLegID,visualLowerLegID,basePosition,baseOrientation,
                          linkMasses=link_Masses,
                          linkCollisionShapeIndices=linkCollisionShapeIndices,
                          linkVisualShapeIndices=linkVisualShapeIndices,
                          linkPositions=linkPositions,
                          linkOrientations=linkOrientations,
                          linkInertialFramePositions=linkInertialFramePositions,
                          linkInertialFrameOrientations=linkInertialFrameOrientations,
                          linkParentIndices=indices,
                          linkJointTypes=jointTypes,
                          linkJointAxis=axis)

p.setGravity(0,0,-10)
#p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
for i in range (10000):
  p.stepSimulation()
  time.sleep(1./240.)
#time.sleep(100)
p.disconnect()


  
