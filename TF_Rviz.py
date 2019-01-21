#from pyquaternion import Quaternion
import numpy as np
import time
import rospy
import roslib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf

def callback(data):
    global tf
    #print(data)
    br.sendTransform(
            (data.transform.translation.x,
            data.transform.translation.y,
            data.transform.translation.z),
            (data.transform.rotation.x,
            data.transform.rotation.y,
            data.transform.rotation.z,
            data.transform.rotation.w),
            rospy.Time.now(),'Ground',"world")

rospy.init_node('tf_broadcaster')
rospy.Subscriber("/vicon/firefly_sbx/firefly_sbx",TransformStamped,callback)
br = tf.TransformBroadcaster()
rospy.spin()
