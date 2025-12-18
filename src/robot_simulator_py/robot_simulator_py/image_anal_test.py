#!/usr/bin/env python3

import math
from typing import Dict, Tuple
import cv2
import numpy as np
import time


image1 = cv2.imread('/home/rex/Documents/ROBE313/ros2_ws_herter_mcallister_ward/src/robot_simulator_py/robot_simulator_py/image1_movement.png')
image2 = cv2.imread('/home/rex/Documents/ROBE313/ros2_ws_herter_mcallister_ward/src/robot_simulator_py/robot_simulator_py/image2_movement.png')


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()
aruco_params.adaptiveThreshWinSizeMin = 3
aruco_params.adaptiveThreshWinSizeMax = 23
aruco_params.adaptiveThreshConstant = 7
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX



# img1 = cv2.imgmsg_to_cv2(image1, "bgr8")
# img2 = cv2.imgmsg_to_cv2(image2, "bgr8")
gray1 = cv2.cvtColor(image1, cv2.COLOR_BGR2GRAY)
gray2 = cv2.cvtColor(image2, cv2.COLOR_BGR2GRAY)

diff = cv2.absdiff(gray1, gray2)
thresh = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)[1]
thresh = cv2.dilate(thresh, None, iterations=2)

contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

if contours:
    largest_contour = max(contours, key=cv2.contourArea)
    height, width = gray1.shape
    M = cv2.moments(largest_contour)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(height) - int(M["m01"] / M["m00"])
        print(f"movement at x: {cx}, y: {cy}")





