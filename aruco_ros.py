import os
import cv2
import json
import sys
import numpy as np
import time
import cv2.aruco as aruco

import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

calibration = json.load(open('cam_bluefox2.json'))
cameraMatrix = np.array(calibration['cameraMatrix'])
distCoeffs = np.array(calibration['distCoeffs'])

# Constant parameters used in Aruco methods
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)

# Create grid board object we're using in our stream
board = aruco.GridBoard_create(markersX=5,markersY=7,markerLength=2.28/100,markerSeparation=0.61/100,dictionary=ARUCO_DICT)


class Recv_Node():
    def __init__(self,src_topic,node_name='Container_Node'):
        rospy.init_node(node_name)
        self.src_topic = src_topic
        self.src_compressed = True if self.src_topic.endswith('/compressed') else False
        self.br = tf.TransformBroadcaster()
        if self.src_compressed:
            self.sub = rospy.Subscriber(self.src_topic, CompressedImage, self.image_callback, queue_size=1)
        else:
            self.sub = rospy.Subscriber(self.src_topic, Image, self.image_callback, queue_size=1)
    

    def image_callback(self,ros_data):
        if self.src_compressed:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr,1)
        else:
            cv_image = bridge.imgmsg_to_cv2(ros_data)
        self.process_image(cv_image)


    def publish_tf(self,rvec,tvec):
        quaternion = tf.transformations.quaternion_from_euler(rvec[0][0],rvec[1][0],rvec[2][0])
        self.br.sendTransform(tvec,quaternion,rospy.Time.now(),'bluefox',"world")


    def process_image(self,QueryImg):
        gray = cv2.cvtColor(QueryImg, cv2.COLOR_BGR2GRAY)
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
                self.publish_tf(rvec,tvec)

        cv2.imshow('QueryImage', QueryImg)
        cv2.waitKey(1)

def main(_):
    src_topic = '/mv_26806344/image_raw/compressed'
    node = Recv_Node(src_topic)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv[1:])