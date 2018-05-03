#!/usr/bin/python2.7
import rospy 
import roslib
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import socket
import time

rospy.init_node('pos_socket_recv')
subimu = rospy.Publisher('another/pos', Odometry, queue_size=1)
s =  socket.socket(AF_INET,SOCK_DGRAM)
addr = ('127.0.0.1',10001)
s.bind(addr)

rate = rospy.Rate(10) 
while rospy.is_shutdown():
    data,addr = udpServer.recvfrom(1024)
    print len(data.encode(encoding="utf-8"))
    rate.sleep()
sock.close()
