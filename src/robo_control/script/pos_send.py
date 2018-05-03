#!/usr/bin/python2.7
import rospy 
import roslib
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import socket
import time


pos_x=pos_y=pos_yaw=0

def callback_odom(msg):
    global pos_x,pos_y,pos_qx,pos_q,pos_qz,pos_qw
    pos_x=msg.pose.pose.position.x
    pos_y=msg.pose.pose.position.y
    qua = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(qua)
    pos_yaw=yaw

rospy.init_node('pos_socket_send')
subimu = rospy.Subscriber('odom', Odometry, callback_odom)
s =  socket.socket(AF_INET,SOCK_DGRAM)
addr = ('127.0.0.1',10001)

rate = rospy.Rate(10) 
while not rospy.is_shutdown():
    data_send="%f  %f   %f"%(pos_x,pos_y,pos_yaw)
    print data_send
    s.sendto(data_send.encode(encoding="utf-8"),addr) 
    rate.sleep()
s.close()