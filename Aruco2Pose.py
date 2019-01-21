import os
import cv2
import json
import numpy as np
import time
import cv2.aruco as aruco

import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf

calibration = json.load(open('calibration.json'))
cameraMatrix = np.array(calibration['cameraMatrix'])
distCoeffs = np.array(calibration['distCoeffs'])

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)

# Create grid board object we're using in our stream
board = aruco.GridBoard_create(
        markersX=5,markersY=7,
        markerLength=2.28/100,markerSeparation=0.61/100,
        dictionary=ARUCO_DICT)

rospy.init_node('aruco_node')
br = tf.TransformBroadcaster()
def publish_tf(rvec,tvec):
    quaternion = tf.transformations.quaternion_from_euler(rvec[0][0],rvec[1][0],rvec[2][0])
    br.sendTransform(tvec,quaternion,rospy.Time.now(),'Ground',"world")

cam = cv2.VideoCapture(0)

if __name__ == '__main__':
    while(cam.isOpened()):
        ret, QueryImg = cam.read()
        if ret == True:
            gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)
        
            # Detect Aruco markers
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    
            # Refine detected markers
            # Eliminates markers not part of our board, adds missing markers to the board
            corners, ids, rejectedImgPoints, recoveredIds = aruco.refineDetectedMarkers(
                    image = gray,
                    board = board,
                    detectedCorners = corners,
                    detectedIds = ids,
                    rejectedCorners = rejectedImgPoints,
                    cameraMatrix = cameraMatrix,
                    distCoeffs = distCoeffs)   

            QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, borderColor=(0, 0, 255))

            # Require 15 markers before drawing axis
            if ids is not None and len(ids) > 10:
                # Estimate the posture of the gridboard, which is a construction of 3D space based on the 2D video 
                retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs)
                if retval:
                    QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 0.1)
                    publish_tf(rvec,tvec)

            cv2.imshow('QueryImage', QueryImg)

        if cv2.waitKey(1)==27:
            break