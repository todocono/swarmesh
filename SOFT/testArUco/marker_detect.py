from cam_calib.camera_calibration import cam_calib
import numpy as np
import cv2 as cv
from cv2 import aruco
import math
import json


# ################### CALCULATE ############################################
def calc_orientation(vector):
    tan = vector[0][1] / vector[0][0]
    ori = math.degrees(math.atan(tan) * 2)
    return round(ori, 0)


def calc_pos(corners):
    pos = [int(sum([j[0] for j in corners[0]]) // 4) - 240,
           int(sum([k[1] for k in corners[0]]) // 4)]
    return pos


# coef is 0.21
def cvt_pos(coord):
    return [round(i * 0.21, 0) for i in coord]


# ################### INITIALIZING ############################################
rt, mtx, dist, cam_rvecs, cam_tvecs = cam_calib()  # to run calibration
# mtx = np.array([[1.1551e+03, 0, 5.457e+02],
#                [0, 1.1509e+03, 7.174e+02],
#                [0, 0, 1]])  # calibrate result
# dist = np.array([3.932e-1, -2.104, -3.6e-3, 1.9e-3, 3.1098])
# calculated from Matlab
# mtx = np.array([[6.204675e+02, 0, 3.601938e+02],
#                [0, 6.19458e+02, 2.450647e+02],
#                [0, 0, 1]])  # calibrate result
# dist = np.array([1.043e-01, -1.524e-01, 1.3e-3, 1e-3, -1.15e-01])

source = 0  # "http://ZDRM:12345678@10.209.31.55:8081" #0 or 1 for usb webcam
cap = cv.VideoCapture(source)
cap.set(3, 1920)
cap.set(4, 1080)
font = cv.FONT_HERSHEY_SIMPLEX
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
params = aruco.DetectorParameters_create()

while not cap:
    cap = cv.VideoCapture(source)
print("press q to quit.")
while True:
    ret, frame = cap.read()
    h, w = frame.shape[:2]
    new_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    dst = cv.undistort(frame, mtx, dist, None, new_mtx)
    gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
    gray = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 11, 2)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                          aruco_dict,
                                                          parameters=params)
    if ids is not None:
        pos_dict = {}
        for n in range(len(ids)):  # we analyze the markers found
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[n],
                                                              0.05, mtx, dist)
            id = ids[n][0]
            pos = calc_pos(corners[n])
            ori = calc_orientation(rvecs[0])
            aruco.drawAxis(dst, mtx, dist, rvecs[0], tvecs[0], 0.05)
            pos = cvt_pos(pos)
            pos_dict[int(id)] = [pos, ori]
            # x_center = (corners[n][0][0][0] + corners[n][0][2][0])/2
            # y_center = (corners[n][0][0][1] + corners[n][0][2][1])/2
            # x_center2 = (corners[n][0][1][0] + corners[n][0][3][0])/2
            # y_center2 = (corners[n][0][1][1] + corners[n][0][3][1])/2
            # print(str(ids[n]) +
            #      # " 1st corner at " + str(corners[n][0][0]) +
            #      # " 2nd at " + str(corners[n][0][1]) +
            #      # " 3rd at " + str(corners[n][0][2]) +
            #      # " 4th at " + str(corners[n][0][3]) +
            #       " with center at x= " + str(x_center) +
            #       " y= " + str(y_center) +
            #       " with center2 at x= " + str(x_center2) +
            #       " y= " + str(y_center2))
        print(pos_dict)
        for i in corners:  # corners[i][0][j][0]  and  corners[i][0][k][1]
            pos = (int(sum([j[0] for j in i[0]]) / 4),
                   int(sum([k[1] for k in i[0]]) // 4))
            txt_pos = pos[0], pos[1] - 20
            cv.circle(dst, pos, 1, (0, 0, 255), -1)
            cv.putText(dst, '{}'.format(pos), txt_pos, font, 0.3, (0, 255, 0))
            # print(pos)

        # for i in range(len(ids)):
        #     aruco.drawAxis(dst, mtx, dist, rvecs[0], tvecs[0], 0.05)
    # cv.namedWindow('dst', 0)
    # cv.imshow("orig", frame)
    imS = cv.resize(dst, (960, 480))
    cv.imshow("dst", dst)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
