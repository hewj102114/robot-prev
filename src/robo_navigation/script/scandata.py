#!/usr/bin/env python
import rospy 
import roslib
import cv2
import numpy as np
import matplotlib.pyplot as plt 
from sensor_msgs.msg import LaserScan

def callback_scan(data):
    rho=data.ranges
    #print 'rho'
    theta=np.arange(0,2*np.pi,np.pi/180)
    #plt.figure()
    plt.subplot(121,polar=True) 
    plt.plot(theta,rho,lw=2)
    plt.plot(theta[19],rho[19],lw=10)
    plt.draw()
    plt.pause(0.0000000001)
    plt.clf()


if __name__ == '__main__':
    rospy.init_node('scandata')

    sublidar = rospy.Subscriber('/scan', LaserScan, callback_scan)
    plt.ion()
    plt.show()
    rospy.spin()




