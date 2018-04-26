import rospy 
import roslib
import cv2
import numpy as np

rospy.init_node('scandata')

sublidar = rospy.Subscriber('/scan', scan, callback_scan)



