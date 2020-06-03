from cam_calib.camera_calibration import cam_calib
import numpy as np
import cv2 as cv
from cv2 import aruco

#################### INITIALIZING ##############################################
rt, mtx, dist, cam_rvecs, cam_tvecs = cam_calib() 
# mtx = np.array([[1.1551e+03, 0, 5.457e+02],[0, 1.1509e+03, 7.174e+02],[0,0,1]]) #calibrate result
# dist = np.array([3.932e-1, -2.104,-3.6e-3,1.9e-3,3.1098])
# source = "http://ZDRM:12345678@10.209.31.55:8081"n  
source = 1  
cap = cv.VideoCapture(source)
font = cv.FONT_HERSHEY_SIMPLEX
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
params = aruco.DetectorParameters_create()
while not cap:
    cap = cv.VideoCapture(source)
print("press q to quit.")
while True:
    ret, frame = cap.read()
    h,w = frame.shape[:2]
    new_mtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv.undistort(frame, mtx, dist, None, new_mtx)
    gray = cv.cvtColor(dst, cv.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, \
                                                         aruco_dict, \
                                                         parameters=params)
    if ids is not None:
        aruco.drawDetectedMarkers(dst, corners, ids, (0,0,255))
        for i in corners:
            pos = (int(sum([j[0] for j in i[0]])/4),\
                    int(sum([k[1] for k in i[0]])//4))
            txt_pos = pos[0], pos[1] - 20
            cv.circle(dst, pos, 1, (0,0,255),-1) 
            cv.putText(dst,'{}'.format(pos),txt_pos, font,0.3,(0,255,0))
            # print(pos)
                
        rvecs, tvecs, _= aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        for i in range(len(ids)):
            aruco.drawAxis(dst, mtx, dist, rvecs[i], tvecs[i], 0.05)
    # cv.namedWindow('dst', 0)
    cv.imshow("orig", frame)
    cv.imshow("dst", dst)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break


    