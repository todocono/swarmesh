import numpy as np
import cv2 as cv
from cv2 import aruco, aruco_Board, aruco_Dictionary
import random
from matplotlib import pyplot as plt

# fig = plt.figure()
# fig.suptitle("No Title")
# fig, ax_lst = plt.subplots(2,2)
plt.figure()
row,col = 4, 5

for i in range(1,21):
    marker = np.zeros((100,100,3),np.uint8)
    d_marker = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    marker = d_marker.drawMarker(i,400,marker,1)
    plt.subplot(row,col,i)
    plt.xticks([])
    plt.yticks([])
    plt.imshow(marker)
    # plt.imread(marker)
    # window = plt.imshow(marker)    
    # cv.waitKey(0)
plt.suptitle("Aruco Markers")
plt.show()
    
