import os
import cv2
import json
import numpy as np
import time
import cv2.aruco as aruco

# Check for camera calibration data
"""if not os.path.exists('calibration.pckl'):
    print("You need to calibrate the camera you'll be using. See calibration project directory for details.")
    exit()
else:
    f = open('calibration.pckl', 'rb')
    (cameraMatrix, distCoeffs) = pickle.load(f)
    f.close()
    if cameraMatrix is None or distCoeffs is None:
        print("Calibration issue. Remove ./calibration.pckl and recalibrate your camera with CalibrateCamera.py.")
        exit()"""
"""calibration={
    'cameraMatrix':cameraMatrix.tolist(),
    'distCoeffs':distCoeffs.tolist()
}
json.dump(calibration,open('calibration.json','w'),indent=4)"""

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

#cam = cv2.VideoCapture('gridboard-test-at-kyles-desk2.mp4')
cam = cv2.VideoCapture(0)
while(cam.isOpened()):
    ret, QueryImg = cam.read()
    if ret == True:
        start = time.time()
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

        ###########################################################################
        # TODO: Add validation here to reject IDs/corners not part of a gridboard #
        ###########################################################################

        # Outline all of the markers detected in our image
        QueryImg = aruco.drawDetectedMarkers(QueryImg, corners, borderColor=(0, 0, 255))

        # Require 15 markers before drawing axis
        if ids is not None and len(ids) > 10:
            # Estimate the posture of the gridboard, which is a construction of 3D space based on the 2D video 
            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs)
            if retval:
                QueryImg = aruco.drawAxis(QueryImg, cameraMatrix, distCoeffs, rvec, tvec, 0.1)
                rotation,jacobian = cv2.Rodrigues(rvec)
                #print(retval)
                #print(rotation)
                #print(rvec.tolist())
                #print(tvec.tolist())
        print(1000*(time.time()-start))
        cv2.imshow('QueryImage', QueryImg)

    if cv2.waitKey(1)==27:
        break

cv2.destroyAllWindows()
