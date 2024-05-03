import numpy as np
import pybullet as p
import time
import pybullet_data
import math
from copy import deepcopy
DURATION = 800
ALPHA = 400

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
print("data path: %s " % pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0.5, 0.5, 0]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, np.pi/2])
boxId = p.loadURDF("husky/husky.urdf", cubeStartPos, cubeStartOrientation,globalScaling = 0.6)
r = c = 5
cone_id = [[0 for _ in range(r)] for __ in range(c)]
for i in range(r):
    for j in range(c):
        cone_id[i][j] = p.loadURDF("cone.urdf", [
                   i, j, 0],  p.getQuaternionFromEuler([np.pi/2, 0, 0]),useFixedBase=1, globalScaling = 0.005)
# gemId = p.loadURDF("cone.urdf", [
#                    3, 0, 0],  p.getQuaternionFromEuler([np.pi/2, 0, 0]),useFixedBase=1, globalScaling = 0.01)
# print(np.array(p.getBasePositionAndOrientation(gemId)[0]))
with open('action.txt') as f:
    contents = f.readlines()
print(contents)
path=contents[0]
newPos=[]
for action in path:
    boxPos, boxOrn = p.getBasePositionAndOrientation(boxId)
    boxPos = list(boxPos)
    boxOrn=list(p.getEulerFromQuaternion(boxOrn))
    match action:
        case 'R':
            if(boxOrn[2]>np.pi/4):
                for i in range(100):
                    boxOrn[2]-=np.pi/(2*100)
                    p.resetBasePositionAndOrientation(boxId, boxPos, p.getQuaternionFromEuler(boxOrn))
                    p.stepSimulation()
                    time.sleep(.001)
                p.resetBasePositionAndOrientation(boxId, boxPos, p.getQuaternionFromEuler([0,0,0]))
                p.stepSimulation()
            newPos=deepcopy(boxPos)
            newPos[0]+=1
            vel=3

        case 'L':
            if(boxOrn[2]>np.pi/4):
                for i in range(100):
                    boxOrn[2]-=np.pi/(2*100)
                    p.resetBasePositionAndOrientation(boxId, boxPos, p.getQuaternionFromEuler(boxOrn))
                    p.stepSimulation()
                    time.sleep(.001)
                p.resetBasePositionAndOrientation(boxId, boxPos, p.getQuaternionFromEuler([0,0,0]))
                p.stepSimulation()
            newPos=deepcopy(boxPos)
            newPos[0]-=1
            vel=-3

        case 'U':
            if(round(boxOrn[2])==0):
                for i in range(100):
                    boxOrn[2]+=np.pi/(2*100)
                    p.resetBasePositionAndOrientation(boxId, boxPos, p.getQuaternionFromEuler(boxOrn))
                    p.stepSimulation()
                    time.sleep(.001)
                p.resetBasePositionAndOrientation(boxId, boxPos, p.getQuaternionFromEuler([0,0,np.pi/2]))
                p.stepSimulation()
            newPos=deepcopy(boxPos)
            newPos[1] +=1
            vel=3

        case 'D':
            if(round(boxOrn[2])==0):
                for i in range(100):
                    boxOrn[2]+=np.pi/(2*100)
                    p.resetBasePositionAndOrientation(boxId, boxPos, p.getQuaternionFromEuler(boxOrn))
                    p.stepSimulation()
                    time.sleep(.001)
                p.resetBasePositionAndOrientation(boxId, boxPos, p.getQuaternionFromEuler([0,0,np.pi/2]))
                p.stepSimulation()
            newPos=deepcopy(boxPos)
            newPos[1] -=1
            vel=-3
                 
            
    for i in range(DURATION):
        # if(i<1200):
        #     continue
        p.stepSimulation()
        #time.sleep(1./240.)
        #gemPos, gemOrn = p.getBasePositionAndOrientation(gemId)
        boxPos, boxOrn = p.getBasePositionAndOrientation(boxId)
        # vel=3
        #print(boxPos[0])
        # newPos=np.array(boxPos)
        # newPos[0]+=1
        print(newPos)
        dist = np.linalg.norm(np.array(boxPos)-np.array(newPos))
        print(dist)
        if(dist<.01):
            for joint in range(2, 6):
                    p.setJointMotorControl2(boxId, joint,p.VELOCITY_CONTROL,targetVelocity=0,
                                            force = 1000000000)
            p.resetBasePositionAndOrientation(boxId, newPos, boxOrn)
            # break
        else:
            for joint in range(2, 6):
                    p.setJointMotorControl2(boxId, joint,p.VELOCITY_CONTROL,targetVelocity=vel*2,
                                            force = 10)
            
        # p.setForce(objectUniqueId=boxId, linkIndex=-1,
        #                      forceObj=force, posObj=boxPos, flags=p.WORLD_FRAME)
        #print(boxPos)
# p.disconnect()
print(contents)