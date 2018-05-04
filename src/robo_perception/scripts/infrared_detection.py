#!/usr/bin/python2.7
#coding=utf-8
####!/usr/bin/python2.7
###!/home/ubuntu/anaconda2/bin/python2.7
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import roslib
import time
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import cv2
import sys
import os
import glob
import tf
import tf2_ros

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from robo_perception.msg import ObjectList
from robo_perception.msg import Object

import tensorflow as tf
from config import *
from train import _draw_box
from nets import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

frame_rate_list = np.zeros(10)
count = 0
frame_rate_idx = 0
frame_rate = 0.0
flag = True
qn_odom = [0, 0, 0, 0]
team_x = team_y = team_relative_x = team_relative_y = 0
odom_pos_x = odom_pos_y = 0

video = cv2.VideoWriter('/home/ubuntu/robot/src/robo_perception/scripts/visual/demo.avi',
                        cv2.VideoWriter_fourcc(*"MJPG"),
                        20,
                        (848, 480))

def TFinit():
    global tfBuffer
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

def DetectInit():
    global sess, model, mc

    detect_net = 'squeezeDet'
    checkpoint = '/home/ubuntu/robot/src/robo_perception/scripts/weights/Infrared-Image/model.ckpt-99000'

    assert detect_net == 'squeezeDet' or detect_net == 'squeezeDet+', 'Selected nueral net architecture not supported'

    tf.Graph().as_default()
    # Load model
    if detect_net == 'squeezeDet':
        mc = kitti_squeezeDet_config()
        mc.BATCH_SIZE = 1
        # model parameters will be restored from checkpoint
        mc.LOAD_PRETRAINED_MODEL = False
        model = SqueezeDet(mc, '0')
    elif detect_net == 'squeezeDet+':
        mc = kitti_squeezeDetPlus_config()
        mc.BATCH_SIZE = 1
        mc.LOAD_PRETRAINED_MODEL = False
        model = SqueezeDetPlus(mc, '0')

    saver = tf.train.Saver(model.model_params)
    # Use jit xla
    gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.5)
    config = tf.ConfigProto(allow_soft_placement=True, gpu_options=gpu_options)
    config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
    # with tf.Session(config=config) as sess:
    sess = tf.Session(config=config)
    saver.restore(sess, checkpoint)

def enemy_self_identify(rgb_image, robo_bboxes, show_image=False):
    enemy = []
    for robo_bbox in robo_bboxes:
        cx, cy, w, h = robo_bbox
        resize_const = 620.0
        aligned_w = w * (848.0 / resize_const)
        aligned_h = h * (848.0 / resize_const)
        aligned_cx = (cx - 424) * (848.0 / resize_const) + 424.0 + 15.0
        alinged_cy = (cy - 240) * (848.0 / resize_const) + 240.0

        aligned_robo_bbox = [aligned_cx, alinged_cy, aligned_w, aligned_h]
        x_min = int(aligned_robo_bbox[0] - aligned_robo_bbox[2]/2)
        x_max = int(aligned_robo_bbox[0] + aligned_robo_bbox[2]/2)
        y_min = int(aligned_robo_bbox[1] - aligned_robo_bbox[3]/2)
        y_max = int(aligned_robo_bbox[1] + aligned_robo_bbox[3]/2)
        # rgb -> bgr
        robo_image = rgb_image[y_min:y_max, x_min:x_max, ::-1]
        h_, w_, c_ = robo_image.shape
        # print(robo_image)
        b_channel = np.sum(robo_image[int(h_/2):-1,:,0])
        g_channel = np.sum(robo_image[int(h_/2):-1,:,1])
        r_channel = np.sum(robo_image[int(h_/2):-1,:,2])
        if b_channel < r_channel:
            judge = 1       # enemy
        else:
            judge = 0       # self
        if show_image:
            cv2.rectangle(rgb_image, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255,0,0), 2)
            # cv2.rectangle(im, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (255,0,0), 2)
            cv2.imshow('rgb', rgb_image)
            cv2.imshow('robo_image', robo_image)
        enemy.append(judge)
    return enemy

def TsDet_callback(infrared_image, pointcloud):
    print("====================================new image======================================")
    print("===================================================================================")
    global count, sess, model, mc, video, frame_rate_list, frame_rate_idx, frame_rate, align_image
    print('I here rgb and pointcloud !', count)
    count = count + 1

    bridge = CvBridge()
    try:
        cv_image_rgb = bridge.imgmsg_to_cv2(
            infrared_image, desired_encoding="8UC1")
        cv_image_rgb = cv2.cvtColor(cv_image_rgb, cv2.COLOR_GRAY2RGB)
    except CvBridgeError as error:
        print(error)

    times = {}
    t_start = time.time()

    im = cv_image_rgb
    im = im.astype(np.float32, copy=False)
    im = cv2.resize(im, (mc.IMAGE_WIDTH, mc.IMAGE_HEIGHT))
    input_image = im - mc.BGR_MEANS

    t_reshape = time.time()
    times['reshape'] = t_reshape - t_start

    # 使用SqueezeDet进行目标检测
    det_boxes, det_probs, det_class = sess.run([model.det_boxes, model.det_probs, model.det_class],
                                               feed_dict={model.image_input: [input_image]})
    # 获取检测时间
    t_detect = time.time()
    times['detect'] = t_detect - t_reshape

    # 进行 NMS 运算
    final_boxes, final_probs, final_class = model.filter_prediction(det_boxes[0],
                                                                    det_probs[0],
                                                                    det_class[0])
    # 按照阈值筛选bbox
    keep_idx = np.squeeze(np.argwhere(
        np.array(final_probs) > mc.PLOT_PROB_THRESH))

    # 得到筛选后的bbox
    final_boxes = np.array(final_boxes)[keep_idx, :]
    final_probs = np.array(final_probs)[keep_idx]
    final_class = np.array(final_class)[keep_idx]

    # 如果没有检测到任何物体, 对存储检测结果的list进行变换
    if keep_idx.shape == ():
        final_boxes = final_boxes[np.newaxis, :]
        final_probs = np.array([final_probs])
        final_class = np.array([final_class])

    avgX, avgY, avgZ = 0, 0, 0
    robo_position = []
    roi = [[0, 0], [0, 0]]
    rois = []
    if mc.DEBUG:
        print("bboxes: %s, probs: %s, class: %s" %
              (final_class, final_probs, final_class))

    # 0 -> red, 1 -> wheel, 2 -> blue
    # 检测到车并且检测到轮子, 进入此 if, 目的是为了减少误检率
    robo_bboxes = []
    if len(final_boxes) > 0 and np.any(final_class == 0) and np.any(final_class == 1):
        #judge detect how much robots
        robot_final_idx = np.array(np.where(final_class != 1))
        for _ in range(robot_final_idx.shape[1]):
            robo_idx = robot_final_idx[0, _]
            robo_bbox = final_boxes[robo_idx, :]
            cx, cy, w, h = robo_bbox[0], robo_bbox[1], robo_bbox[2], robo_bbox[3]   # 获取车bbox的坐标, 宽度和高度
            robo_bboxes.append(robo_bbox)

            # 用于提取点云的范围大小
            pointcloud_w = 5
            pointcloud_h = 5

            # 对点云进行约束
            if pointcloud_w > robo_bbox[2]:
                pointcloud_w = robo_bbox[2]
            if pointcloud_h > robo_bbox[3]:
                pointcloud_h = robo_bbox[3]

            # 使用中心位置的 pointcloud_w*pointcloud_h 的点云计算距离
            #x_ = np.arange(int(cx - pointcloud_w/2), int(cx + pointcloud_w/2), 1)
            #y_ = np.arange(int(cy - pointcloud_h/2), int(cy + pointcloud_h/2), 1)

            # 使用bbox下方的点云计算距离, 大概是装甲板的位置
            x_ = np.arange(int(cx - pointcloud_w / 2),
                        int(cx + pointcloud_w / 2), 1)
            y_ = np.arange(int(cy + h / 2 - h / 6 - pointcloud_h),
                        int(cy + h / 2 - h / 6), 1)
            roi = [[x, y] for x in x_ for y in y_]
            rois.append(roi)
            # 提取特定位置的点云
            points = list(pc2.read_points(pointcloud, skip_nans=False,
                                        field_names=("x", "y", "z"), uvs=roi))
            robo_pointcloud = np.array(points)

            # 对提取到的点云进行 reshape
            # TODO: 应该不需要这一步
            positionX = robo_pointcloud[:, 0].reshape(-1, pointcloud_w*pointcloud_h).squeeze()
            positionY = robo_pointcloud[:, 1].reshape(-1, pointcloud_w*pointcloud_h).squeeze()
            positionZ = robo_pointcloud[:, 2].reshape(-1, pointcloud_w*pointcloud_h).squeeze()

            # 剔除距离为 nan 的点
            positionX = positionX[np.logical_not(np.isnan(positionX))]
            positionY = positionY[np.logical_not(np.isnan(positionY))]
            positionZ = positionZ[np.logical_not(np.isnan(positionZ))]

            # 计算距离均值, 得到最终距离
            avgX = np.mean(positionX)
            avgY = np.mean(positionY)
            avgZ = np.mean(positionZ)
            if np.isnan(avgX) or np.isnan(avgY) or np.isnan(avgZ):
                print("continue")                                
                continue
            else:
                robo_position.append([avgX, avgY, avgZ])
            if mc.DEBUG:
                print('enemy position:', avgX, avgY, avgZ)

        enemy_self_list = enemy_self_identify(align_image, robo_bboxes)
        # 检测完敌人, 对得到的距离信息进行处理
        # tf 包转换
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        robo_position = np.array(robo_position)
        enemy_position = ObjectList()
        enemy_position.header.stamp = rospy.Time.now()
        enemy_position.header.frame_id = 'enemy'
        enemy_position.num = robo_position.shape[0]

        # 判断有几个敌人, 1个的时候要特殊处理
        if robo_position.shape[0] == 1:
            robo_position = np.array(robo_position)

        red_idx = 0
        blue_idx = 0
        for object_idx in range(robo_position.shape[0]):
            # ROS 中发送数据
            enemy = Object()
            enemy.pose.position.x = robo_position[object_idx, 2]
            enemy.pose.position.y = -robo_position[object_idx, 0]
            enemy.pose.position.z = robo_position[object_idx, 1]
            enemy_position.object.append(enemy)

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'realsense_camera'
            if enemy_self_list[object_idx] == 1:    # enemy(red)
                str_enemy_self = 'red'
                t.child_frame_id = str_enemy_self + str(red_idx)
                enemy.team.data = str_enemy_self + str(red_idx)
                red_idx = red_idx + 1
            else:
                str_enemy_self = 'blue'
                t.child_frame_id = str_enemy_self + str(blue_idx)
                enemy.team.data = str_enemy_self + str(blue_idx)
                blue_idx = blue_idx + 1

            t.transform.translation.x = robo_position[object_idx, 2]
            t.transform.translation.y = -robo_position[object_idx, 0]
            t.transform.translation.z = robo_position[object_idx, 1]

            t.transform.rotation.x = 0
            t.transform.rotation.y = 0
            t.transform.rotation.z = 0
            t.transform.rotation.w = 1
            br.sendTransform(t)
    else:
        # 如果没有发现敌人
        if mc.DEBUG:
            print('No enemy!!!')
        enemy_position = ObjectList()
        enemy_position.header.stamp = rospy.Time.now()
        enemy_position.header.frame_id = 'enemy'
        enemy_position.num = 0
        enemy_position.object = []
    pub.publish(enemy_position)


    
    t_filter = time.time()
    times['filter'] = t_filter - t_detect
    times['total'] = time.time() - t_start

    time_str = 'Total time: {:.4f}, detection time: {:.4f}, filter time: ' \
               '{:.4f}'. \
        format(times['total'], times['detect'], times['filter'])
    print(time_str)

    # # TODO(bichen): move this color dict to configuration file
    cls2clr = {'robot': (0, 0, 255), 'wheel': (0, 255, 0)}
    if mc.VISUAL:
        im = _draw_box(im,
                       final_boxes,
                       [mc.CLASS_NAMES[idx] + ':%.2f' % prob for idx,
                           prob in zip(final_class, final_probs)],
                       cdict=cls2clr)
        for _ in range(len(robo_position)):
            avgX = robo_position[_, 0]
            avgY = robo_position[_, 1]
            avgZ = robo_position[_, 2]
            position_str = 'x=' + str(round(avgX, 3)) + ' y=' + str(round(avgY, 3)) + ' z=' + str(round(avgZ, 3))
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(im, position_str, (10, 30*(_ + 1)), font, 0.7, (0, 255, 255), 2)
        for _ in range(len(rois)):
            cv2.rectangle(im, (rois[_][0][0], rois[_][0][1]), (rois[_][-1][0], rois[_][-1][1]), (255, 0, 0), 5)

        # 是否录制视频
        if mc.DRAW_Video:
            im = im.astype('uint8')
            video.write(im)
        # 是否存储图像
        if mc.DRAW_BOX:
            image_count = count
            image_name = image_count % mc.SAVE_NUM
            file_name = str(image_name) + '.jpg'
            out_file_name = os.path.join(
                '/home/ubuntu/robot/src/robo_perception/scripts/visual', 'out_' + file_name)
            im = im.astype('uint8')
            cv2.imwrite(out_file_name, im)
        if mc.SHOW:
            im = im.astype('uint8')
            cv2.imshow('demo', im)
            cv2.waitKey(3)

            # Im_to_ros = Image()
            # try:
            #     Im_to_ros = bridge.cv2_to_imgmsg(im, "bgr8")
            # except CvBridgeError as e:
            #     print(e)
            # Im_to_ros.header.stamp = rospy.Time.now()
            # Im_to_ros.header.frame_id = 'camera_link'
            # pub_dr.publish(Im_to_ros)


# def callback_odom(odom):
#     global qn_odom, odom_pos_x, odom_pos_y
#     # only yaw are available
#     odom_pos_x = odom.pose.pose.position.x
#     odom_pos_y = odom.pose.pose.position.y
#     qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
#               odom.pose.pose.orientation.w]
#     # (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)

# def callback_team(team):
#     global team_x, team_y, team_relative_x, team_relative_y, tfBuffer, odom_pos_x, odom_pos_y
#     team_x = team.pose.pose.position.x
#     team_y = team.pose.pose.position.y
#     try:
#         realsense_trans = tfBuffer.lookup_transform('odom', 'realsense_camera', rospy.Time(0))
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
#         print('ENEMY OBJ TRANS FAIL! set enemy number 0')
#     team_relative_x = team_x - realsense_trans.transform.translation.x
#     team_relative_y = team_y - realsense_trans.transform.translation.y
    
#     print ('orirelx',team_relative_x,'orirely', team_relative_y)
#     if team_relative_x > 0:
#         team_relative_x = team_relative_x - 0.2
#     elif team_relative_x < 0:
#         team_relative_x = team_relative_x + 0.2
#     if team_relative_y > 0:
#         team_relative_y = team_relative_y - 0.2
#     elif team_relative_y < 0:
#         team_relative_y = team_relative_y + 0.2

#     teamtheta = np.arctan2(team_relative_y, team_relative_x)
#     teamdistance = np.sqrt(team_relative_x**2 + team_relative_y**2)

#     print ('odom_pos_x',odom_pos_x,'odom_pos_y', odom_pos_y)
#     print ('realsense_x',realsense_trans.transform.translation.x,'realsense_y', realsense_trans.transform.translation.y)
#     print ('relx',team_relative_x,'rely', team_relative_y)
#     print ('team_x',team_x,'team_y', team_y)

def callback_rgb(rgb):
    global align_image
    bridge = CvBridge()
    try:
        align_image = bridge.imgmsg_to_cv2(rgb, desired_encoding="8UC3")

    except CvBridgeError as error:
        print(error)

rospy.init_node('rgb_detection')
TFinit()
DetectInit()
rgb_sub = message_filters.Subscriber('camera/infra1/image_rect_raw', Image)
#subodom = rospy.Subscriber('odom', Odometry, callback_odom)
#subteam = rospy.Subscriber('team/pos', Odometry, callback_team)
subrgb = rospy.Subscriber('/camera/color/image_raw', Image, callback_rgb)

pc_sub = message_filters.Subscriber('camera/points', PointCloud2)
pub = rospy.Publisher('infrared_detection/enemy_position', ObjectList, queue_size=1)

# TODO 在后面的试验中调调整slop
TsDet = message_filters.ApproximateTimeSynchronizer(
    [rgb_sub, pc_sub], queue_size=5, slop=0.1, allow_headerless=False)
TsDet.registerCallback(TsDet_callback)


rospy.spin()
video.release()
cv2.destroyAllWindows()
