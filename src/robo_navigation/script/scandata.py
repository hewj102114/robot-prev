import rospy 
import roslib
import cv2
import numpy as np
#import matplotlib.pyplot as plt 
from sensor_msgs.msg import LaserScan

def callback_scan(data):
    #rho=data.ranges
    print 'rho'
    #theta=np.arrange(0,2*np.pi,1)

    #plt.plot(theta,rho)

    #plt.show()


if __name__ == '__main__':
    rospy.init_node('scandata')

    sublidar = rospy.Subscriber('/scan', LaserScan, callback_scan)

    rospy.spin()






