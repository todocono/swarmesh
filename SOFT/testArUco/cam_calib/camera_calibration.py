# -*- coding: UTF-8 -*- #
# Author: Zander_M
# Time: 一月, 17, 2019
# Title: Camera Calibration
'''
Images taken are from Mi MIX2s camera. Return the calibrate matrix.
Program is copied from openCV-python tutorial.
'''
import glob
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
from cv2 import aruco, aruco_CharucoBoard, aruco_Dictionary
import os


def cam_calib():
    '''
    input: None
    output: rt, mtx, dist, rvecs, tvecs
    description: Return calibrate camera params.
    '''
    ############################ PREPARE ###########################################
    w = 9
    h = 6
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    # prepare cv2 world coordinate as standard base.
    objp = np.zeros((w*h,3), np.float32)
    objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)
    objp = objp * 22  # height and width of chessboard is 22mm
    obj_point = []  # 3d point in real world space
    img_point = []  # 2d points in image plane.

    ############################ IMAGE PROCESS #####################################
    route = os.getcwd() + "\cam_calib\*.jpg"
    print(route)
    images = glob.glob(route)  # route for chessboard photos
    for pic in images:
        img = cv.imread(pic)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        # while True: see if it reads the files
        #     cv.imshow('gray', gray)
        #     cv.waitKey(0)
        #     cv.destroyAllWindows()
        #     break
        ret, corners = cv.findChessboardCorners(gray, (w, h), None)
        if ret:
            # print(pic)
            # plt.imread()
            # cv.namedWindow('img',0)
            obj_point.append(objp)
            corner2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            img_point.append(corner2)
            # cv.drawChessboardCorners(img, (w, h), corner2, ret)  # show corners
            # cv.imshow('img', img)
            # cv.waitKey(500)
            # cv.destroyAllWindows()
    img = cv.imread(images[0])
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    return cv.calibrateCamera(obj_point, img_point, gray.shape[::-1], None, None)


def cam_calib_charuco():
        '''
        camera calibration using CharUco boards.
        '''
        l_corners = []
        l_ids = []
        marker_dict = aruco_Dictionary.get(aruco.DICT_4X4_50)
        corners = []
        board = aruco_CharucoBoard.create(9, 6, 200, 150, marker_dict)
        for images in glob.glob(".\calib\*.jpg"):
            img = cv.imread(images)
            board = aruco_CharucoBoard.create(9, 6, 200, 150, marker_dict)
            corners, ids, = aruco.detectMarkers(img, marker_dict, None, None)
            if len(ids) > 0:
                a, b = aruco.interpolateCornersCharuco(corners, ids, img,
                                                       board, None, None)
                l_corners.append(a)
                l_ids.append(b)
        return aruco.calibrateCameraCharuco(l_corners,\
                                            l_ids,\
                                            board,\
                                            (600, 800),
                                            None,\
                                            None)


def display_calib_rst():
    '''
    show calibrated images
    '''
    images = glob.glob(".\calib\*.jpg")  # route for chessboard photos
    for img_route in images:
        img = cv.imread(img_route)
        h,w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        # dst = cv.undistort(img,cam_param[1],cam_param[2],None, newcammtx)
        mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx,
                                                (w, h), 5)
        dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        # cv.namedWindow('org',0)
        # cv.namedWindow('dst',0)
        cv.imshow("org", img)
        cv.imshow("dst", dst)
        cv.waitKey(0)
        cv.destroyAllWindows()


if __name__ == "__main__":
    rt, mtx, dist, rvecs, tvecs = cam_calib() #cam_calib_charuco()
    # cam_calib_charuco()
    print(mtx)
    print(dist)
    # display_calib_rst()
