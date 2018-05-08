#!/usr/bin/python2.7
import rospy 
import roslib
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import socket
import time


pos_x=pos_y=pos_yaw=0

def callback_odom(msg):
    global pos_x,pos_y,pos_yaw
    pos_x=msg.pose.pose.position.x
    pos_y=msg.pose.pose.position.y
    qua = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(qua)
    pos_yaw=yaw
    

rospy.init_node('pos_socket_send')
subimu = rospy.Subscriber('odom', Odometry, callback_odom)
s =  socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
ip=rospy.get_param('~ip_addr_send','127.0.0.1')
print ip
addr = (ip,10001)

rate = rospy.Rate(60) 
while not rospy.is_shutdown():
    if not pos_x == 0:
        data_send="%f  %f   %f"%(pos_x,pos_y,pos_yaw)
        s.sendto(data_send.encode(encoding="utf-8"),addr) 
    rate.sleep()
s.close()
