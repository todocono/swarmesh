import numpy as np
import time
import cv2
import os
import glob

current_milli_time = lambda: int(round(time.time() * 1000))
w = 9
h = 6
source = 0
cap = cv2.VideoCapture(source)
while not cap:
    cap = cv2.VideoCapture(source)

print("starting video... ")
print("saving pictures every 3s or every SPACE bar pressed ")


current_time = current_milli_time()
path = os.getcwd() + "\calib\calib  "
# path = "E:/swarm/arUco-master/cam_calib/calib"
i = 0

while True:
    ret, frame = cap.read()
    # cv2.namedWindow('original', 0)
    or2 = cv2.resize(frame, (320, 240))
    cv2.imshow("orig", or2)
    # needed to avoid overflow
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        print("ok, I quit!")
        break
    # take pics either every 3s or every space bar stroke
    if k == 32:
        name = path + str(i + 21) + ".jpg"
        print(name)

        cv2.imwrite(name, frame)
        current_time = current_milli_time()
        i = i + 1
        if i > 20:
            break
print ("done !!!")
cap.release()
cv2.destroyAllWindows()
