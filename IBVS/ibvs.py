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
def run():
    raise NotImplementedError

    # TODO:
    # 1. Subscribe to rostopic camera feed
    # 2. User aurco to detect marker
    # 3. Drive towards goal
    

if __name__ == "__main__":
    run()