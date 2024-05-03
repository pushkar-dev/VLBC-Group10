import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import cv2 as cv
from cv2 import aruco

i = 25
def getJacobian(u, v, f=1, z=1):
    J = [[-f/z, 0, u/z, u*v/z, -(f + u*u)/f, v], [0, -f/z, v/z, (f+v*v)/f, -u*v/f, -u]]
    return J

def imageConversion(arr, h, w):
    width, height = w, h
    global i
    pixels = []
    for y in range(height*width):
        r, g, b, a = arr[4*y], arr[4*y+1], arr[4*y+2], arr[4*y+3]
        pixels.append((r, g, b, a))

    # Create a new image from the pixel values
    img = Image.new('RGBA', (width, height))
    img.putdata(pixels)
    # img.save(f"output_pbvs.png")
    i += 1
    return np.array(img)

def servoing(frame, h, w):

    marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
    param_markers = aruco.DetectorParameters_create()
    
    global i

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        print("here")
        ids, corners = marker_IDs[0], marker_corners[0]
        cv.polylines(
            frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
        )
        corners = corners.reshape(4, 2)
        corners = corners.astype(int)
        top_right = list(corners[0].ravel())
        top_left = list(corners[1].ravel())
        bottom_right = list(corners[2].ravel())
        bottom_left = list(corners[3].ravel())
        points = [top_left, top_right, bottom_left, bottom_right]
        points.sort()
        if points[0][1] > points[1][1]:
            points[0][1], points[1][1] = points[1][1], points[0][1]
        if points[2][1] > points[3][1]:
            points[2][1], points[3][1] = points[3][1], points[2][1]
        centre=top_left
        centre[0]=int(round((top_left[0]+top_right[0]+bottom_left[0]+bottom_right[0])/4))
        centre[1]=int(round((top_left[1]+top_right[1]+bottom_left[1]+bottom_right[1])/4))
        cv.putText(
                frame,
                f"xyz",
                centre,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv.LINE_AA,
            )
            #print(ids, "  ", corners)
        # cv.imshow("frame", frame)
        # key = cv.waitKey(1)
        # cv.imwrite(f"debug_d{i}.png", frame)
        # cap.release()
        return points

fr = cv.imread("output_reference.png")
requiredPos = servoing(fr, 600, 800)

def getpv(newpoints):
    a = [newpoints[2][i]-newpoints[0][i] for i in range(3)]
    b = [newpoints[1][i]-newpoints[0][i] for i in range(3)]
    return np.cross(b, a)

def transformAxes(transform, coordinates):
    if(len(transform[0])==4):
        a = np.ones(4)
    else:
        a = np.ones(3)
    a[:3] = coordinates
    # print(transform, a, coordinates)
    a = np.matmul(transform, a)
    a = a[:3]
    return a

def robotControl(points, f=1, k=None):
    # global requiredPos
    # # print(requiredPos, points)
    # error = []
    # for i in range(3):
    #     for j in range(2):
    #         error.append(requiredPos[i][j] - points[i][j])
    # error = np.array(error)
    # J = []
    # z = [1, 1, 1]
    # if k is not None:
    #     for i in range(3):
    #         z[i] = k[points[i][1]*800+points[i][0]]
    # J += getJacobian(*points[0], z[0])
    # J += getJacobian(*points[1], z[1])
    # J += getJacobian(*points[2], z[2])
    # j = np.array(J)
    # J_1 = np.linalg.inv(J)
    # velocity = np.matmul(J_1, error)
    
    # return velocity
    newpoints = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    print(len(k))
    print(points)
    for i in range(4):
        z = k[points[i][1]*800+points[i][0]]
        newpoints[i][0] = points[i][0]*z/f
        newpoints[i][1] = points[i][1]*z/f
        newpoints[i][2] = z/f
    
    return newpoints



# @jit(target_backend="cuda")
def main():
    physicsClient = p.connect(p.GUI)
    dt = 0.0003
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # print(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    cubeStartPos = [0, 0, 1]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, np.pi/4])
    boxId = p.loadURDF("husky/husky.urdf", cubeStartPos, cubeStartOrientation)
    textureId = p.loadTexture("6x6.png")
    orn = [0, 0, 0]
    
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, np.pi/4])
    obstacleId = p.loadURDF("cube_small.urdf", [cubeStartPos[0], cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)
    # obstacleId1 = p.loadURDF("cube_small.urdf", [cubeStartPos[0]+1, cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)
    # obstacleId2 = p.loadURDF("cube_small.urdf", [cubeStartPos[0]-1, cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    
    p.changeVisualShape(objectUniqueId=obstacleId,
                     linkIndex=-1, 
                     textureUniqueId=textureId)
    useRealTimeSimulation = 1
    base_link_id = p.getBodyInfo(boxId)[0]
    #print(base_link_id)
    if (useRealTimeSimulation):
        p.setRealTimeSimulation(1)

    sleep(1)
    i = 0
    
    if (useRealTimeSimulation):
        p.stepSimulation()
        # p.setGravity(0, 0, -10)
        basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
        cubePos, cubeOrn = p.getBasePositionAndOrientation(obstacleId)
        rot_matrix = p.getMatrixFromQuaternion(baseOrn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # print(rot_matrix)
        # Initial vectors
        init_camera_vector = (1, 0, 0) # z-axis
        init_up_vector = (0, 0, 1) # y-axis
        # Rotated vectors
        basePos2 = [basePos[0], basePos[1], basePos[2]+0.7]
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(basePos2, basePos + 2 * camera_vector, up_vector)
        sleep(1)
        
        
        width, height = 800, 600
        
        fov = 90


        width, height = 800, 600
        aspect = width / height
        nearVal = 0.01
        farVal = 100
        projection_matrix = p.computeProjectionMatrixFOV(
            fov, aspect, nearVal, farVal
        )
        k = p.getCameraImage(width, height,view_matrix,projection_matrix)
        arr = np.array(k[2])
        arr = imageConversion(arr, height, width)
        points = servoing(arr, height, width)
        # if points is None:
        #     basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
        #     baseOrn = list(p.getEulerFromQuaternion(baseOrn))
        #     baseOrn[2] += np.pi/18
        #     p.resetBasePositionAndOrientation(boxId, basePos, p.getQuaternionFromEuler(baseOrn))
        #     continue
        print(f"debug: {points}")
        newpoints = robotControl(points, 50, k[3])

        rot_matrix = np.linalg.inv(rot_matrix)
        transform = np.zeros((4, 4))
        for i in range(3):
            for j in range(3):
                transform[i][j] = rot_matrix[i][j]

        basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
        transform[0][3] = basePos[0]
        transform[1][3] = basePos[1]
        transform[2][3] = basePos[2]
        transform[3][3] = 1

        tCamRobot = np.zeros((4, 4))
        tCamRobot[0][2] = 1
        tCamRobot[1][0] = -1
        tCamRobot[2][1] = -1
        tCamRobot[0][3] = basePos[0]
        tCamRobot[1][3] = basePos[1]
        tCamRobot[2][3] = basePos[2]+.7
        tCamRobot[3][3] = 1

        for i in range(len(newpoints)):
            newpoints[i] = transformAxes(tCamRobot, newpoints[i])
            # newpoints[i] = transformAxes(transform, newpoints[i])
        
        print(newpoints)

        for i in range(len(newpoints)):
            # newpoints[i] = transformAxes(tCamRobot, newpoints[i])
            newpoints[i] = transformAxes(transform, newpoints[i])
        
        print(newpoints)

        pv = getpv(newpoints)

        centre = [0, 0, 0]
        for i in range(3):
            for j in range(4):
                centre[i] += newpoints[j][i]
            centre[i] /= 4
        
        ln = np.linalg.norm(pv)
        newPos = [0, 0, 0]

        for i in range(2):
            newPos[i] = 0.2*pv[i]/ln + centre[i]
        

        # centreHomogenous = [i for i in centre]
        # centreHomogenous.append(1)

        # centreHomogenous = np.array(centreHomogenous)
        print(newPos)
        velocity = [0, 0, 0]
        basePos, _ = p.getBasePositionAndOrientation(boxId)
        ln = np.linalg.norm([basePos[i]-newPos[i] for i in range(3)])
        time = ln/(0.2)
        for i in range(2):
            velocity[i] = (basePos[i]-newPos[i])/(time)
        #print(time)
        dist = np.linalg.norm(np.array(basePos)-np.array(newPos))
        print(newPos)
        if(dist<0.5):
            for joint in range(2, 6):
                    p.setJointMotorControl2(boxId, joint,p.VELOCITY_CONTROL,targetVelocity=0,
                                            force = 10)
        else:
            for joint in range(2, 6):
                    p.setJointMotorControl2(boxId, joint,p.VELOCITY_CONTROL,targetVelocity=3,
                                            force = 10)
        print(p.getBasePositionAndOrientation(boxId)[0])
        
        # time /= 2
        # while time>0:
        #     time -= 1
        #     basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
        #     basePos1, baseOrn1 = p.getBasePositionAndOrientation(boxId)
        #     cubePos, cubeOrn = p.getBasePositionAndOrientation(obstacleId)
        #     rot_matrix = p.getMatrixFromQuaternion(baseOrn)
        #     rot_matrix = np.array(rot_matrix).reshape(3, 3)
        #     # print(rot_matrix)
        #     # Initial vectors
        #     init_camera_vector = (1, 0, 0) # z-axis
        #     init_up_vector = (0, 0, 1) # y-axis
        #     # Rotated vectors
        #     basePos2 = [basePos[0], basePos[1], basePos[2]+0.7]
        #     camera_vector = rot_matrix.dot(init_camera_vector)
        #     up_vector = rot_matrix.dot(init_up_vector)
        #     view_matrix = p.computeViewMatrix(basePos2, basePos + 2 * camera_vector, up_vector)
        #     sleep(1)
            
        #     p.stepSimulation()
        #     width, height = 800, 600
            
        #     fov = 90


        #     width, height = 800, 600
        #     aspect = width / height
        #     nearVal = 0.01
        #     farVal = 100
        #     projection_matrix = p.computeProjectionMatrixFOV(
        #         fov, aspect, nearVal, farVal
        #     )
        #     k = p.getCameraImage(width, height,view_matrix,projection_matrix)

        #     basePos, _ = p.getBasePositionAndOrientation(boxId)
        #     basePos = list(basePos)
        #     for i in range(3):
        #         basePos[i] += velocity[i]
        #     print(basePos)
        #     p.resetBasePositionAndOrientation(boxId, basePos, baseOrn)
            
        #     dist = np.linalg.norm(np.array(basePos1)-np.array(basePos))
        #     print(dist)
        #     if(dist<0.15):
        #         for joint in range(2, 6):
        #                 p.setJointMotorControl2(boxId, joint,p.VELOCITY_CONTROL,targetVelocity=0,
        #                                         force = 10)
        #     else:
        #         for joint in range(2, 6):
        #                 p.setJointMotorControl2(boxId, joint,p.VELOCITY_CONTROL,targetVelocity=3,
        #                                         force = 10)
        #     p.stepSimulation()
        #     sleep(.5)
        #     #print("X = {:+f}, Y = {:+f}, Z = {:+f}, Roll = {:+f}, Pitch = {:+f}, Yaw = {:+f}".format(*pos,*rpy))
        print("done")
        sleep(20)
    else:
        # basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
        # p.changeConstraint(boxId, [basePos[0], basePos[1]-10, basePos[2]+5], baseOrn)
        # p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=[basePos[0], basePos[1]-10, basePos[2]+5]) 
        # p.stepSimulation()
        pass

if __name__ == "__main__":
    main()