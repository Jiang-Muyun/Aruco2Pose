import os
import cv2
import json
import sys
import time
import numpy as np

import rospy
import roslib
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Recv_Node():
    def __init__(self,src_topic,node_name='Container_Node'):
        rospy.init_node(node_name)
        self.src_topic = src_topic
        self.src_compressed = True if self.src_topic.endswith('/compressed') else False
        if self.src_compressed:
            self.sub = rospy.Subscriber(self.src_topic, CompressedImage, self.image_callback, queue_size=1)
        else:
            self.sub = rospy.Subscriber(self.src_topic, Image, self.image_callback, queue_size=1)
        self.index = 0

    def image_callback(self,ros_data):
        if self.src_compressed:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr,1)
        else:
            cv_image = bridge.imgmsg_to_cv2(ros_data)

        cv2.imshow('cv_image', cv_image)
        key = cv2.waitKey(1)
        if 113 == key or 27 == key:
            rospy.signal_shutdown(0)
        elif 32 == key:
            print('Take photo',self.index)
            cv2.imwrite('./calib_photos/img_%d.jpg'%(self.index),cv_image)
            self.index += 1
        elif key != 255:
            print(key)

def main(_):
    src_topic = '/mv_26806344/image_raw/compressed'
    node = Recv_Node(src_topic)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv[1:])