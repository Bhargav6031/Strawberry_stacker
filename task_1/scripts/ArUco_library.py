#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time


def detect_ArUco(img):
    # function to detect ArUco markers in the image using ArUco library
    # argument: img is the test image
    # return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
    # for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
    # {0: array([[315, 163],
    #							[319, 263],
    #							[219, 267],
    #							[215,167]], dtype=float32)}

    aruco_list = {}
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters)
    if len(corners):
        for k in range(len(corners)):
            temp_1 = corners[k]
            temp_1 = temp_1[0]
            temp_2 = ids[k]
            temp_2 = temp_2[0]
            aruco_list[temp_2] = temp_1
    print(aruco_list)
    return aruco_list


def Calculate_orientation_in_degree(Detected_ArUco_markers):
    # function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
    # argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
    # return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
    # for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the
    # function should return: {1: 120 , 2: 164}

    ArUco_marker_angles = {}

    ## enter your code here ##
    angle_list = list(range(359, 0, -1))
    key_list = Detected_ArUco_markers.keys()
    for key in key_list:
        dict_entry = Detected_ArUco_markers[key]
        centre = dict_entry[0].astype(
            int) + dict_entry[1].astype(int) + dict_entry[2].astype(int) + dict_entry[3].astype(int)
        centre[:] = [int(x / 4) for x in centre]
        centre = tuple(centre)
        orient_centre = (dict_entry[0].astype(int)+dict_entry[1].astype(int))/2
        orient_centre = tuple(orient_centre.astype(int))
        pt1, pt2 = tuple(centre), tuple(orient_centre)
        x = pt2[0]-pt1[0]
        y = pt2[1]-pt1[1]
        angle = int(math.degrees(math.atan2(y, x)))
        angle = angle_list[angle]
        ArUco_marker_angles[key] = angle
    # returning the angles of the ArUco markers in degrees as a dictionary
    return ArUco_marker_angles

def read_chessboards(images):
    #reading the chessboard
    print("POSE ESTIMATION STARTED...")
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        # print("=> Processing image {0}".format(im))
        print('.', end="")
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)

        if len(corners)>0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize = (3,3),
                                 zeroZone = (-1,-1),
                                 criteria = criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners,ids,gray,board)
            if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator+=1

    imsize = gray.shape
    return allCorners,allIds,imsize

def mark_ArUco(img, Detected_ArUco_markers, ArUco_marker_angles):
    # function to mark ArUco in the test image as per the instructions given in problem statement
    # arguments: img is the test image
    # Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
    # ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
    # return: image namely img after marking the aruco as per the instruction given in problem statement

    ## enter your code here ##
    font = cv2.FONT_HERSHEY_SIMPLEX
    key_list = Detected_ArUco_markers.keys()
    for key in key_list:
        dict_entry = Detected_ArUco_markers[key]
        centre = dict_entry[0].astype(
            int) + dict_entry[1].astype(int) + dict_entry[2].astype(int) + dict_entry[3].astype(int)
        centre[:] = [int(x / 4) for x in centre]
        centre = tuple(centre)
        orient_centre = (dict_entry[0].astype(int)+dict_entry[1].astype(int))/2
        orient_centre = tuple(orient_centre.astype(int))
        cv2.circle(img, centre, 1, (0, 0, 255), 15)
        cv2.circle(img, tuple(dict_entry[0].astype(
            int)), 1, (125, 125, 125), 15)
        cv2.circle(img, tuple(dict_entry[1].astype(int)), 1, (0, 255, 0), 15)
        cv2.circle(img, tuple(dict_entry[3].astype(
            int)), 1, (255, 255, 255), 15)
        cv2.circle(img, tuple(dict_entry[2].astype(
            int)), 1, (180, 105, 255), 15)
        angle = ArUco_marker_angles[key]
        cv2.line(img, centre, orient_centre, (255, 0, 0), 4)
        cv2.putText(img, str(angle), (int(
            centre[0] - 100), int(centre[1])), font, 1.5, (0, 255, 0), 4, cv2.LINE_AA)
        cv2.putText(img, str(key), (int(
            centre[0] + 50), int(centre[1])), font, 1.5, (0, 0, 255), 4, cv2.LINE_AA)

        return img
