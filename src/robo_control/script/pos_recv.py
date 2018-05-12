#!/usr/bin/python2.7
import rospy
import roslib
from nav_msgs.msg import Odometry
from robo_control.msg import TeamInfo
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import socket
import time


RECV_LEN = 9
lost_counter = 0
LOST_TRESH = 50

rospy.init_node('pos_socket_recv')
pub_team = rospy.Publisher('team/info', TeamInfo, queue_size=1)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
ip = rospy.get_param('~ip_addr_recv', '127.0.0.1')
addr = (ip, 10001)
s.bind(addr)

rate = rospy.Rate(50)
while not rospy.is_shutdown():
    global lost_counter
    data, addr = s.recvfrom(1024)
    datalist = data.split()
    team = TeamInfo()
    if len(datalist) == RECV_LEN:
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

        
        team.header.frame_id = "team"
        team.header.stamp = rospy.Time.now()

        team.remainingHP = remainingHP
        team.bulletCount = bulletCount

        team.pose.pose.position.x = pos_x
        team.pose.pose.position.y = pos_y
        team.pose.pose.orientation.x = qua[0]
        team.pose.pose.orientation.y = qua[1]
        team.pose.pose.orientation.z = qua[2]
        team.pose.pose.orientation.w = qua[3]

        team.targetRelative.pose.position.x = target_global_x
        team.targetRelative.pose.position.y = target_global_y

        team.targetGlobal.pose.position.x = target_rel_x
        team.targetGlobal.pose.position.y = target_rel_y


        lost_counter = 0
    else:
        lost_counter = lost_counter + 1
        print lost_counter

#connection = 0 temperal lost
#connection = 1 forever lost
#connection = 2 connection work fine
    if lost_counter != 0:
        team.connection = 0
    if lost_counter > LOST_TRESH:
        team.connection = 1
    if lost_counter == 0:
        team.connection = 2

    
    pub_team.publish(team)
    rate.sleep()
s.close()
