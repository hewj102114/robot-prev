#!/usr/bin/python2.7
import rospy
import roslib
from nav_msgs.msg import Odometry
from robo_control.msg import TeamInfo
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import socket
import time


SEND_LEN = 9

rospy.init_node('pos_socket_recv')
pub_pos = rospy.Publisher('team/pos', Odometry, queue_size=1)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = rospy.get_param('~ip_addr_recv', '127.0.0.1')
addr = (ip, 10001)
s.bind(addr)

rate = rospy.Rate(50)
while not rospy.is_shutdown():
    data, addr = s.recvfrom(1024)
    datalist = data.split()
    if len(datalist) == SEND_LEN:
        pos_x = float(datalist[0])
        pos_y = float(datalist[1])
        pos_yaw = float(datalist[2])
        target_global_x = float(datalist[3])
        target_global_y = float(datalist[4])
        target_rel_x = float(datalist[5])
        target_rel_y = float(datalist[6])
        remainingHP = int(datalist[7])
        bulletCount = int(datalist[8])

        qua = quaternion_from_euler(0, 0, pos_yaw)

        team = TeamInfo()
        team.header.frame_id = "team"
        team.header.stamp = rospy.Time.now()
        team.pose.pose.position.x = pos_x
        team.pose.pose.position.y = pos_y
        team.pose.pose.orientation.x = qua[0]
        team.pose.pose.orientation.y = qua[1]
        team.pose.pose.orientation.z = qua[2]
        team.pose.pose.orientation.w = qua[3]
        pub_team.publish(team)
        pos_x, pos_y, pos_yaw, target_global_x, target_global_y, target_rel_x, target_rel_y, remainingHP, bulletCount
    rate.sleep()
s.close()
