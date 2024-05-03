import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import cv2 as cv
from cv2 import aruco

i = 25
def getJacobian(u, v, f=1, z=5):
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
    # img.save(f"output_reference.png")
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
rmse=[]
timestamp = []
def robotControl(points, k=None):
    global requiredPos
    global rmse
    # print(requiredPos, points)
    error = []
    rms=0
    for i in range(3):
        for j in range(2):
            error.append(requiredPos[i][j] - points[i][j])
            rms += (-requiredPos[i][j] + points[i][j])**2
    error = np.array(error)
    rms = rms**0.5
    rmse.append(rms)
    timestamp.append(len(timestamp))
    J = []
    z = [1, 1, 1]
    if k is not None:
        for i in range(3):
            z[i] = k[points[i][1]*800+points[i][0]]
    J += getJacobian(*points[0], z[0])
    J += getJacobian(*points[1], z[1])
    J += getJacobian(*points[2], z[2])
    j = np.array(J)
    J_1 = np.linalg.inv(J)
    velocity = np.matmul(J_1, error)
    # for i in range(len(velocity)):
    #     velocity[i] *=-1
    return velocity


# @jit(target_backend="cuda")
def main():
    physicsClient = p.connect(p.GUI)
    dt = 0.0003
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # print(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    cubeStartPos = [-2, 0, 1]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, np.pi/4])
    boxId = p.loadURDF("husky/husky.urdf", cubeStartPos, cubeStartOrientation)
    textureId = p.loadTexture("6x6.png")
    orn = [0, 0, 0]

    cubeStartPos = [0, 0, 1]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    obstacleId = p.loadURDF("cube_small.urdf", [cubeStartPos[0], cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)
    obstacleId1 = p.loadURDF("cube_small.urdf", [cubeStartPos[0]+1, cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)
    obstacleId2 = p.loadURDF("cube_small.urdf", [cubeStartPos[0]-1, cubeStartPos[1]+5, cubeStartPos[2]], cubeStartOrientation, globalScaling = 20)
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
    while 1:
        i += 1
        if (useRealTimeSimulation):
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
            
            #print(baseOrn)
            #print(type(boxId), type(base_link_id), type(p.LINK_FRAME))
            # p.applyExternalForce(boxId, -1, force, position, p.LINK_FRAME)
            # baseOrn = p.getEulerFromQuaternion(baseOrn)
            # p.resetBasePositionAndOrientation(obstacleId, cubePos, p.getQuaternionFromEuler(orn))
            # print(p.getEulerFromQuaternion(baseOrn))
            # p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=-10, cameraPitch=-10, cameraTargetPosition=[basePos[0], basePos[1]-1, basePos[2]+1]) 
            p.stepSimulation()
            # orn[2] += np.pi/18
            
            width, height = 800, 600
            # view_matrix = p.computeViewMatrix(
            #     cameraEyePosition=[basePos[0], basePos[1], basePos[2]+1],
            #     cameraTargetPosition=[basePos[0], basePos[1]+1, basePos[2]+1],
            #     cameraUpVector=[0, 0, 1]
            # )
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
            # view_matrix = p.computeViewMatrix(
            #     cameraEyePosition=[basePos[0], basePos[1]-1, basePos[2]+1],
            #     cameraTargetPosition=[0, 100, 0],
            #     cameraUpVector=[0, 1, 0]
            # )
            # fov = 90

            # width, height = 400, 400
            # aspect = width / height
            # nearVal = 0.01
            # farVal = 100
            # projection_matrix = p.computeProjectionMatrixFOV(
            #     fov, aspect, nearVal, farVal
            # )
            # k = p.getCameraImage(width, height,view_matrix,projection_matrix)
            arr = imageConversion(k[2], height, width)
            points = servoing(arr, height, width)
            # print(points)
            if not points:
                # p.setRealTimeSimulation(0)
                print("Done")
                _, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
                baseOrn = p.getEulerFromQuaternion(baseOrn)
                p.resetBasePositionAndOrientation(boxId, basePos, p.getQuaternionFromEuler([baseOrn[0], baseOrn[1], baseOrn[2]+np.pi/18]))
                continue
            else:
                velocity = robotControl(points)
            # print(basePos, baseOrn, velocity, sep = '\n')

            #ax.plot(timestamp, rmse, color='r')
            plt.plot(timestamp, rmse,color='black')
        
            plt.title(f'Cummaltive error )')
            plt.xlabel('instance')
            plt.ylabel('error')
            

            plt.tight_layout()
            plt.gcf().canvas.draw() #update display window
            plt.pause(0.01)

            basePos = list(basePos)
            baseOrn = list(p.getEulerFromQuaternion(baseOrn))
            
            tCamRobot = np.zeros((4, 4))
            tCamRobot[0][2] = 1
            tCamRobot[1][0] = -1
            tCamRobot[2][1] = -1
            tCamRobot[0][3] = basePos[0]
            tCamRobot[1][3] = basePos[1]
            tCamRobot[2][3] = basePos[2]+.7
            tCamRobot[3][3] = 1
            print(f"vel before edit: {velocity}")
            rot = velocity[-1]

            velocity[3] = 1
            print(velocity)

            delPos = np.matmul(tCamRobot, velocity[:4])
            
            transform = np.zeros((4, 4))
            #rot_matrix_inv = np.linalg.inv(rot_matrix)
            for kl in range(3):
                for mn in range(3):
                    transform[kl][mn] = rot_matrix[kl][mn]
            # transform = np.linalg.inv(transform)


            transform[0][3] = basePos[0]
            transform[1][3] = basePos[1]
            transform[2][3] = basePos[2]
            transform[3][3] = 1

            print(f"transform{transform}")
            velocity[3] = 1

            delPos = np.matmul(transform, delPos)
            print(f"velocity: {velocity}, delpos:{delPos}", sep='\n')
            basePos[0] += delPos[0]*dt
            basePos[1] += delPos[1]*dt
            # basePos[2] += delPos[2]*dt

            # baseOrn[0] += velocity[3]*dt*50
            # baseOrn[1] += velocity[4]*dt*50
            baseOrn[2] += rot*dt*50
            baseOrn = p.getQuaternionFromEuler(baseOrn)
            p.resetBasePositionAndOrientation(boxId, basePos, baseOrn)
            # p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=-10, cameraPitch=-10, cameraTargetPosition=[basePos[0], basePos[1]-1, basePos[2]+1]) 

            # pos,orientation = p.getBasePositionAndOrientation(boxId)
            # rpy = p.getEulerFromQuaternion(orientation)
            sleep(.01)
            #print("X = {:+f}, Y = {:+f}, Z = {:+f}, Roll = {:+f}, Pitch = {:+f}, Yaw = {:+f}".format(*pos,*rpy))
        else:
            # basePos, baseOrn = p.getBasePositionAndOrientation(boxId) # Get model position
            # p.changeConstraint(boxId, [basePos[0], basePos[1]-10, basePos[2]+5], baseOrn)
            # p.resetDebugVisualizerCamera(cameraDistance=0.3, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=[basePos[0], basePos[1]-10, basePos[2]+5]) 
            # p.stepSimulation()
            pass

if __name__ == "__main__":
    main()