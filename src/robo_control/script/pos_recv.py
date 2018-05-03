#!/usr/bin/python2.7
import rospy 
import roslib
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import socket
import time

rospy.init_node('pos_socket_recv')
pub_pos = rospy.Publisher('another/pos', Odometry, queue_size=1)
s =  socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
ip=rospy.get_param('~ip_addr_recv','127.0.0.1')
addr = (ip,10001)
s.bind(addr)

rate = rospy.Rate(10) 
while not rospy.is_shutdown():
    data,addr = s.recvfrom(1024)
    datalist=data.split()
    if len(datalist)==3:
        pos_x=float(datalist[0])
        pos_y=float(datalist[1])
        pos_yaw=float(datalist[2])
        qua= quaternion_from_euler(0, 0, pos_yaw)
        pos = Odometry()
        pos.header.frame_id = "odom"
        pos.header.stamp=rospy.Time.now() 
        pos.pose.pose.position.x=pos_x
        pos.pose.pose.position.y=pos_y
        pos.pose.pose.orientation.x=qua[0]
        pos.pose.pose.orientation.y=qua[1]
        pos.pose.pose.orientation.z=qua[2]
        pos.pose.pose.orientation.w=qua[3]
        pub_pos.publish(pos)
    rate.sleep()
s.close()
