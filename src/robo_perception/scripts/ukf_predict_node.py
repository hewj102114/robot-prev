#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

###############UKF参数选取和系统方程测定方法##############
#参数选取原则  
#首先按照动力学方程建立F方程 速度等于加速度乘时间。 v（i+1） = v（i） + a*dt
#F要和H相乘，H=【vx,vx',vy,vy'】，所以写为下面的形式，两个yaw分别是imu和uwb的观测
#用H来选择有几个输入参数，这里我有三个参数，分别是yaw观测1，yaw观测2，dyaw。  
#dim_x = 4， 表示输入方程有四个输入  
#dim_z = 4 表示观测方程z有4个观测。两个轴，xy，速度和加速度
#v_std_x, a_std_x, 表示速度和加速度的测量误差，这里根据测量结果填写
#Q噪声项，var这里我发现用0.2的结果比较好，目前还不知道为什么  
#P里头分别填上前面两轴速度加速度能达到的最大值。  【vx,vx',vy,vy'】
#ukf.x = np.array([0., 0., 0., 0.])表示起始位置和起始速度都是0
# MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=-1.0) 4表示输入参数有三个yaw，yaw，dyaw  
#######################################################
#数据融合方法说明：
#when in short distance mode using pnp as the main prediction source, if pnp lost target, use realsense to predict.
#when in long distance mode, using the realsense to predict, and use a state machine to dicede all possible situation.


#注意事项：开车前保持车辆静止
#################比赛场地坐标系定义###########################
#                  X+
#                  1
#                  0
#                  0
#                  0
#                  0
#                  0
#                  0
# Y+  <-000000000000

# Check list
# KALMAN_GAIN是不是对应上去了

import rospy 
import roslib
import pickle
import math
import tf
import tf2_ros
import time
from numpy.random import randn
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from robo_perception.msg import ObjectList
from robo_perception.msg import Object
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

T_DELAY = 0.1 #系统延时系数，单位秒
RS_INIT = True
PNP_INIT = True

ENABLE_PREDICT = True
PNP_LAST_AVAILABLE = False
PNP_DATA_AVAILABLE = False
RS_LAST_AVAILABLE = False
RS_DATA_AVAILABLE = False
RS_PREDICT_INIT = True
PNP_PREDICT_INIT = True

BULLET_SPEED = 17
PNP_CLOSE_THRESH = 0.1 #判断pnp是否瞄准到了正确目标
RS_CLOSE_THRESH = 0.05 #判断rs是否瞄准到了正确目标

ukf_result = []
robo_vel_x = robo_vel_y = 0
pnp_vel_x = pnp_vel_y = pnp_pos_x = pnp_pos_y = last_pnp_pos_x = last_pnp_pos_y = 0
ukf_yaw = ukf_pos_x = ukf_pos_y = ukf_vel_x = ukf_vel_y = 0
rs_pos_x = rs_pos_Y = rs_vel_x = rs_vel_y = last_pnp_pos_x = last_pnp_pos_y = 0
gimbal_roll = gambal_pitch = gimbal_yaw = 0

# S（i+1） = S（i） + V*dt
def f_cv(x, dt):
    """ state transition function for a 
    constant velocity aircraft"""
    
    F = np.array([[1, dt, 0,  0],
                  [0,  1, 0,  0],
                  [0,  0, 1, dt],
                  [0,  0, 0,  1]], dtype=float)
    return np.dot(F, x)

def h_cv(x):
    return np.array([x[0], x[1], x[2], x[3]])


def UKFRsInit(in_dt, init_x):
    global ukf_rs

    p_std_x, p_std_y = 0.03, 0.03
    v_std_x, v_std_y = 0.03, 0.03
    dt = in_dt #50HZ


    sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=-1.0)
    ukf_rs = UKF(dim_x=4, dim_z=4, fx=f_cv, hx=h_cv, dt=dt, points=sigmas)
    ukf_rs.x = init_x
    ukf_rs.R = np.diag([p_std_x, v_std_x, p_std_y, v_std_y]) 
    ukf_rs.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf_rs.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf_rs.P = np.diag([8, 2.0 ,5, 2.0])

def UKFPnpInit(in_dt, init_x):
    global ukf_pnp

    p_std_x, p_std_y = 0.03, 0.03
    v_std_x, v_std_y = 0.03, 0.03
    dt = in_dt #50HZ


    sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=-1.0)
    ukf_pnp = UKF(dim_x=4, dim_z=4, fx=f_cv, hx=h_cv, dt=dt, points=sigmas)
    ukf_pnp.x = init_x
    ukf_pnp.R = np.diag([p_std_x, v_std_x, p_std_y, v_std_y]) 
    ukf_pnp.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf_pnp.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=dt, var=0.2)
    ukf_pnp.P = np.diag([8, 2.0 ,5, 2.0])


def callback_enemy(enemy):
    global enemy_num, team_num, tfBuffer, enemy_object_trans, team_object_trans, aim_target_x, aim_target_y, rs_last_time, rs_lost_time, rs_lost_counter
    global pnp_vel_x, pnp_vel_y, pnp_pos_x, pnp_pos_y, last_pnp_pos_x, last_pnp_pos_y
    global ENABLE_PREDICT, RS_DATA_AVAILABLE, RS_LAST_AVAILABLE, RS_INIT
    if RS_INIT == True:
        print "realsense callback init Finished!"
        rs_last_time = enemy.header.stamp.secs + enemy.header.stamp.nsecs * 10**-9
        enemy_num = 0
        team_object_trans = []
        enemy_object_trans = []
        rs_lost_time = []
        RS_INIT = False
        rs_lost_counter = 0

    elif ENABLE_PREDICT:
        team_object_trans = []
        enemy_object_trans = []
        enemy_num = 0
        team_num = 0
        FIND_RS = False
        rs_time = enemy.header.stamp.secs + enemy.header.stamp.nsecs * 10**-9
        dt = rs_time - rs_last_time

        for i in range(int(enemy.num)):
            object_name =  enemy.object[i].team.data
            if object_name == 'red0' or object_name == 'red1':
                enemy_num = enemy_num + 1
                try:
                    enemy_object_trans.append(tfBuffer.lookup_transform('odom', object_name, rospy.Time(0)))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print('ENEMY OBJ TRANS FAIL! set enemy number 0')
                    enemy_num = 0
                #print enemy_object_trans
            elif object_name == 'blue0':
                team_num = team_num + 1
                try:
                    team_object_trans.append(tfBuffer.lookup_transform('odom', object_name, rospy.Time(0)))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    print('TEAM OBJ TRANS FAIL! set team number 0')
                    team_num = 0

        #object judgement
        if enemy_num == 1:
            enemy_0_x = enemy_object_trans[0].transform.translation.x
            enemy_0_y = enemy_object_trans[0].transform.translation.y
            enemy_1_x = 99
            enemy_1_y = 99
        elif enemy_num == 2:
            enemy_0_x = enemy_object_trans[0].transform.translation.x
            enemy_0_y = enemy_object_trans[0].transform.translation.y
            enemy_1_x = enemy_object_trans[1].transform.translation.x
            enemy_1_y = enemy_object_trans[1].transform.translation.y
        else:
            enemy_0_x = 0
            enemy_0_y = 0
            enemy_1_x = 0
            enemy_1_y = 0
            
        if np.sqrt((aim_target_x - enemy_0_x) ** 2 + (aim_target_x - enemy_0_y) ** 2) < 0.02:
            rs_pos_x = enemy_0_x
            rs_pos_y = enemy_0_y
            FIND_RS = True
        elif np.sqrt((aim_target_x - enemy_1_x) ** 2 + (aim_target_x - enemy_1_y) ** 2) < 0.02:
            rs_pos_x = enemy_1_x
            rs_pos_y = enemy_1_y
            FIND_RS = True
        else:
            # no available data, then not last data.
            FIND_RS = False
            RS_LAST_AVAILABLE = False

            
        # rs position not change
        if  last_rs_pos_x - rs_pos_x == 0:
            RS_LAST_AVAILABLE = False
        
        if FIND_RS == True:
            #print 'USE RS PREDICTING'
            rs_lost_time = []
            rs_lost_counter = 0
            if RS_LAST_AVAILABLE:
                #last data available, use to update the speed
                rs_vel_x = (rs_pos_x - last_rs_pos_x) / dt
                rs_vel_y = (rs_pos_y - last_rs_pos_y) / dt
                RS_DATA_AVAILABLE = True
            
            RS_LAST_AVAILABLE = True
            last_rs_pos_x = rs_pos_x
            last_rs_pos_y = rs_pos_y

        else:
            #print 'NO Available update in realsense ', 'lost_counter:', rs_lost_counter
            rs_lost_counter = rs_lost_counter + 1
            rs_lost_time.append(dt)
            RS_LAST_AVAILABLE = False
            RS_DATA_AVAILABLE = False


        rs_last_time = rs_time


        

def callback_pnp(pnp):
    global pnp_pos_x, pnp_pos_y, aim_target_x, aim_target_y, pnp_lost_counter, last_pnp_pos_x, last_pnp_pos_y, pnp_vel_x, pnp_vel_y
    global pnp_last_time, pnp_lost_time, tfBuffer, pnp_trans
    global PNP_DATA_AVAILABLE, PNP_LAST_AVAILABLE, PNP_INIT, ENABLE_PREDICT
    if PNP_INIT == True:
        print "pnp callback init Finished!"
        pnp_lost_counter = 0
        pnp_last_time = 0
        pnp_lost_time = []
        pnp_trans = TransformStamped()
        PNP_INIT = False   
    elif ENABLE_PREDICT:
        FIND_PNP = False
        pnp_time = pnp.header.stamp.secs + pnp.header.stamp.nsecs * 10**-9
        dt = pnp_time - pnp_last_time
      
        if pnp.pose.position.x != 0 and pnp.pose.position.y != 0:
            FIND_PNP = True
            try:
                pnp_trans = tfBuffer.lookup_transform('odom', 'enemy_pnp_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print('PNP TF TRANS TRANS FAIL! check your code')
            #print pnp_trans
            pnp_pos_x = pnp_trans.transform.translation.x
            pnp_pos_y = pnp_trans.transform.translation.y
        else:
            FIND_PNP = False

        # wrong target judgement   
        if np.sqrt((aim_target_x - pnp_pos_x) ** 2 + (aim_target_x - pnp_pos_y) ** 2) > PNP_CLOSE_THRESH:
            FIND_PNP = False
            PNP_LAST_AVAILABLE = False
        # pnp position not change
        if  last_pnp_pos_x - pnp_pos_x == 0:
            PNP_LAST_AVAILABLE = False
        
        if FIND_PNP == True:
            #print 'USE PNP PREDICTING'
            pnp_lost_time = []
            pnp_lost_counter = 0
            if PNP_LAST_AVAILABLE:
                #last data available, use to update the speed
                pnp_vel_x = (pnp_pos_x - last_pnp_pos_x) / dt
                pnp_vel_y = (pnp_pos_y - last_pnp_pos_y) / dt
                PNP_DATA_AVAILABLE = True
            
            PNP_LAST_AVAILABLE = True
            last_pnp_pos_x = pnp_pos_x
            last_pnp_pos_y = pnp_pos_y

        else:
            #print 'NO Available update in both realsense and gimble camera', 'lost_counter:', pnp_lost_counter
            pnp_lost_counter = pnp_lost_counter + 1
            pnp_lost_time.append(dt)
            PNP_LAST_AVAILABLE = False
            PNP_DATA_AVAILABLE = False

        pnp_last_time = pnp_time
         
        
                
#TODO see weather to combine self speed. or leave it to other code    
def callback_ukf(ukf):
    global ukf_yaw, ukf_pos_x, ukf_pos_y, ukf_vel_x, ukf_vel_y
    #only yaw are available
    ukf_pos_x = ukf.pose.pose.position.x
    ukf_pos_y = ukf.pose.pose.position.y

    ukf_vel_x = ukf.twist.twist.linear.x
    ukf_vel_x = ukf.twist.twist.linear.y

    qn_ukf = [ukf.pose.pose.orientation.x, ukf.pose.pose.orientation.y, ukf.pose.pose.orientation.z, ukf.pose.pose.orientation.w]
    (ukf_roll,ukf_pitch,ukf_yaw) = euler_from_quaternion(qn_ukf)

def TFinit():
    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

def callback_target(target):
    global aim_target_x, aim_target_y, ENABLE_PREDICT
    aim_target_x = target.pose.pose.position.x
    aim_target_y = target.pose.pose.position.y
    ENABLE_PREDICT = target.pose.pose.position.z

rospy.init_node('ukf_predict_node')
UKFRsInit(0.02,np.array([0., 0., 0., 0.]))
UKFPnpInit(0.02,np.array([0., 0., 0., 0.]))
TFinit()
subenemy = rospy.Subscriber('infrared_detection/enemy_position', ObjectList, callback_enemy)
subpnp = rospy.Subscriber('base/armor_pose', PoseStamped, callback_pnp)
subukf = rospy.Subscriber('ukf/pos', Odometry, callback_ukf)
subtarget = rospy.Subscriber('enemy/target', Odometry, callback_target)

pub_ukf_vel = rospy.Publisher('ukf/enemy', Odometry, queue_size=1)

rate = rospy.Rate(50) # 10hz
while not rospy.is_shutdown():
    global BULLET_SPEED, pnp_pos_x, pnp_vel_x, pnp_pos_y, pnp_vel_y, pnp_vel_x, pnp_vel_y, pnp_pos_x, pnp_pos_y, gimbal_roll, gambal_pitch, gimbal_yaw, ukf_vel_x, ukf_vel_y

    #rs有数据就用rs的进行更新，第一次直接初始化，第二次再更新   
    if RS_DATA_AVAILABLE and RS_PREDICT_INIT:
        UKFRsInit(0.02, np.array([rs_pos_x, rs_vel_x, rs_pos_y, rs_vel_y]))
        ukf_rs_pos_x = ukf_rs.x[0]
        ukf_rs_pos_y = ukf_rs.x[2]
        ukf_rs_vel_x = ukf_rs.x[1]
        ukf_rs_vel_y = ukf_rs.x[3]  
        RS_PREDICT_INIT == False
    elif RS_DATA_AVAILABLE:
        rs_ukf_input = [rs_pos_x, rs_vel_x, rs_pos_y, rs_vel_y]
        ukf_rs.predict()
        ukf_rs.update(rs_ukf_input)
        ukf_rs_pos_x = ukf_rs.x[0]
        ukf_rs_pos_y = ukf_rs.x[2]
        ukf_rs_vel_x = ukf_rs.x[1]
        ukf_rs_vel_y = ukf_rs.x[3] 

    #rs有数据就用rs的进行更新，rs没数据就看pnp有没有数据，第一次直接初始化，第二次再更新
    if RS_DATA_AVAILABLE == False and PNP_PREDICT_INIT and PNP_DATA_AVAILABLE:
        UKFPnpInit(0.02, np.array([pnp_pos_x, pnp_vel_x, pnp_pos_y, pnp_vel_y]))
        ukf_pnp_pos_x = ukf_pnp.x[0]
        ukf_pnp_pos_y = ukf_pnp.x[2]
        ukf_pnp_vel_x = ukf_pnp.x[1]
        ukf_pnp_vel_y = ukf_pnp.x[3]  
        PNP_PREDICT_INIT == False
    elif RS_DATA_AVAILABLE == False and PNP_DATA_AVAILABLE:
        pnp_ukf_input = [pnp_pos_x, pnp_vel_x, pnp_pos_y, pnp_vel_y]
        ukf_pnp.predict()
        ukf_pnp.update(pnp_ukf_input)
        ukf_pnp_pos_x = ukf_pnp.x[0]
        ukf_pnp_pos_y = ukf_pnp.x[2]
        ukf_pnp_vel_x = ukf_pnp.x[1]
        ukf_pnp_vel_y = ukf_pnp.x[3] 

    if RS_DATA_AVAILABLE == False:
        RS_PREDICT_INIT = True 
    if PNP_DATA_AVAILABLE == False:
        PNP_PREDICT_INIT = True 

    if RS_DATA_AVAILABLE:
        ukf_out_pos_x = ukf_rs_pos_x 
        ukf_out_pos_y = ukf_rs_pos_y 
        ukf_out_vel_x = ukf_rs_vel_x 
        ukf_out_vel_y = ukf_rs_vel_y 
    elif: PNP_DATA_AVAILABLE:
        ukf_out_pos_x = ukf_pnp_pos_x 
        ukf_out_pos_y = ukf_pnp_pos_y 
        ukf_out_vel_x = ukf_pnp_vel_x 
        ukf_out_vel_y = ukf_pnp_vel_y
    else:
        print 'unable to predict!' 

    #print 'PNP','X',ukf_input[0],'Y',ukf_input[2]
    print 'PNP','VX',ukf_input[1],'VY',ukf_input[3]
    print 'KALMAN', 'VX',ukf_rs.x[1],'VY', ukf_rs.x[3]


    try:
        gimbal_trans = tfBuffer.lookup_transform('odom', 'gimbal_link', rospy.Time(0))
        qn_gimbal = [gimbal_trans.transform.rotation.x, gimbal_trans.transform.rotation.y, gimbal_trans.transform.rotation.z, gimbal_trans.transform.rotation.w]
        (gimbal_roll,gambal_pitch,gimbal_yaw) = euler_from_quaternion(qn_gimbal)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print('GIMBAL TRANS FAIL!')

    #   #
    #           #
    #                 #
    #                        #
    #                               #
    #                                     #
    ########################################
    
    # distance_to_enemy
    # Bullet flying time = 17m/s

    if PNP_DATA_AVAILABLE or RS_DATA_AVAILABLE:
        #计算相对速度
        relative_speed_x = ukf_out_vel_x - ukf_vel_x
        relative_speed_y = ukf_out_vel_y - ukf_vel_y
        #计算水平于枪口方向的速度            
        V_verticle = relative_speed_x * np.cos(gimbal_yaw) + relative_speed_y * np.sin(gimbal_yaw)
        #print V_verticle, ukf_yaw
        #print 'cos',ukf_out_vel_x * np.cos(ukf_yaw),'sin',ukf_out_vel_y * np.sin(ukf_yaw)
        #计算检测到的目标和我自身的距离
        distance_to_enemy = np.sqrt((ukf_out_pos_x - ukf_pos_x)**2 +(ukf_out_pos_y - ukf_pos_y)**2)
        #计算子弹飞行时间
        T_FLY = distance_to_enemy / BULLET_SPEED
        #反解算出需要的预瞄角度
        predict_angle = np.arctan(V_verticle * (T_DELAY + T_FLY) / distance_to_enemy)

    predict_pos = Odometry()
    predict_pos.header.frame_id = "predict"
    predict_pos.header.stamp = rospy.Time.now()
    predict_pos.pose.pose.position.x = ukf_out_pos_x
    predict_pos.pose.pose.position.y = ukf_out_pos_y
    predict_pos.twist.twist.linear.x = ukf_out_vel_x
    predict_pos.twist.twist.linear.y = ukf_out_vel_y

    predict_pos.pose.pose.orientation.y = 0    
    predict_pos.pose.pose.orientation.y = 0
    predict_pos.pose.pose.orientation.z = 0
    predict_pos.pose.pose.orientation.w = predict_angle

    pub_ukf_vel.publish(predict_pos) 

    rate.sleep()