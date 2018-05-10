#include <iostream>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include "robo_vision/AngleSolver.hpp"
#include "robo_vision/Armor.h"
#include "robo_vision/ArmorDetector.hpp"
#include "robo_vision/ArmorInfo.h"
#include "robo_vision/Predictor.hpp"
#include "robo_vision/RMVideoCapture.hpp"
#include "robo_vision/RuneResFilter.hpp"
#include "robo_vision/Settings.hpp"
using namespace std;
using namespace cv;

Mat img_recv;
int pre_seq = 0, cur_seq = 0;
geometry_msgs::Pose enemy_pose;
geometry_msgs::Vector3 pre_enemy_pose;
geometry_msgs::Vector3 pre_angle;

void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    Mat img_temp = cv_bridge::toCvShare(msg, "bgr8")->image;
    // ROS_INFO("recv");
    if (img_temp.empty())
    {
        ROS_ERROR("Could't not get image");
    }
    else
    {
        // cv::imshow("view", img_temp);
        img_recv = img_temp.clone();
        cur_seq = msg->header.seq;
    }
}

void cb_enemy(const nav_msgs::Odometry &msg) { enemy_pose = msg.pose.pose; }
int main(int argc, char *argv[])
{
    // ros init
    ros::init(argc, argv, "robo_vision");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Subscriber sub_enemy = nh.subscribe("enemy/target", 1, &cb_enemy);
    // ros publisher
    ros::Publisher pub_armor_info =
        nh.advertise<robo_vision::ArmorInfo>("base/armor_info", 1);
    ros::Publisher pub_armor_pose =
        nh.advertise<geometry_msgs::PoseStamped>("base/armor_pose", 1);
    tf2_ros::TransformBroadcaster enemy_pnp_tf;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub =
        it.subscribe("usbcamera/image", 1, &image_callback);
    // load setting from fil
    string config_file_name =
        "/home/ubuntu/robot/src/robo_vision/param/param_config.xml";
    private_nh.getParam("config_file_name", config_file_name);
    Settings setting(config_file_name);

    // angle offset
    double offset_anlge_x = 0;
    double offset_anlge_y = 0;
    private_nh.getParam("offset_anlge_x", offset_anlge_x);
    private_nh.getParam("offset_anlge_y", offset_anlge_y);

    // get robot id & choose the calibration file
    string intrinsic_file;
    private_nh.getParam("intrinsic_file", intrinsic_file);
    cout << intrinsic_file << endl;
    FileStorage fs(intrinsic_file, FileStorage::READ);
    if (!fs.isOpened())
    {
        ROS_ERROR("Could not open the configuration file: %s", intrinsic_file);
        return 0;
    }
    Mat cam_matrix_480, distortion_coeff_480;
    fs["Camera_Matrix"] >> cam_matrix_480;
    fs["Distortion_Coefficients"] >> distortion_coeff_480;

    // camera and gimbal transform
    const double ptz_camera_y = 5;
    const double ptz_camera_z = -15;
    double r_data[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double t_data[] = {
        0, ptz_camera_y,
        ptz_camera_z}; // ptz org position in camera coodinate system
    Mat t_camera_ptz(3, 1, CV_64FC1, t_data);
    Mat r_camera_ptz(3, 3, CV_64FC1, r_data);

    AngleSolver angle_solver(cam_matrix_480, distortion_coeff_480, t_camera_ptz,
                             r_camera_ptz, 14.5, 6.2);
    Point2f image_center = Point2f(cam_matrix_480.at<double>(0, 2),
                                   cam_matrix_480.at<double>(1, 2));

    double pre_angle_x = 0.0, pre_angle_y = 0.0;

    // load armor detector setting
    ArmorDetector armor_detector(setting.armor);

    FilterZ filter_z(0.1);
    ArmorFilter armor_filter(7);

    Mat src, src_csm;
    int miss_detection_cnt = 0;

    VideoWriter writer("/home/ubuntu/VideoTest.avi",
                       CV_FOURCC('M', 'J', 'P', 'G'), 25, Size(640, 480));

    ros::Rate rate(150);
    ROS_INFO("Image Consumer Start!");
    tf::TransformListener* tf_ = new tf::TransformListener();

    // armor_detector_flag
    int armor_detector_flag = 0;
    while (ros::ok())
    {
        // ROS_INFO("loop");
        if (img_recv.empty() || pre_seq == cur_seq)
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }
        pre_seq = cur_seq;
        img_recv.copyTo(src);
        src.copyTo(src_csm);

        // msg init
        robo_vision::ArmorInfo msg_armor_info;
        msg_armor_info.header.stamp = ros::Time::now();
        msg_armor_info.header.frame_id = "base_link";

        vector<ArmorTarget> armor_target;
        double angle_x = 0.0, angle_y = 0.0;
        ros::Time start = ros::Time::now();
        armor_detector.getAllTargetAera(src, armor_target);
        msg_armor_info.armor_count = armor_target.size();

        if (armor_target.size() != 0)
        {
            miss_detection_cnt = 0;
            float distance = 10000, idx = -1;
            for (int i = 0; i < armor_target.size(); i++)
            {
                robo_vision::Armor armor_msg;
                angle_solver.getAngle(armor_target[i], angle_x, angle_y);
                armor_msg.pose.x =
                    angle_solver.position_in_camera.at<double>(0, 0)/100.0;
                armor_msg.pose.y =
                    angle_solver.position_in_camera.at<double>(1, 0)/100.0;
                armor_msg.pose.z =
                    angle_solver.position_in_camera.at<double>(2, 0)/100.0;
                armor_msg.angle.x = (angle_x + offset_anlge_x) * 100.0;
                armor_msg.angle.y = (angle_y + offset_anlge_y) * 100.0;

                // tf::StampedTransform transform;
                // try
                // {
                //     tf_listener.lookupTransform("usb_camera_link", "odom",
                //                              ros::Time(0), transform);
                // }
                // catch (tf::TransformException &ex)
                // {
                //     ROS_ERROR("%s", ex.what());
                //     break;
                // }

                tf::Stamped<tf::Pose> ident(tf::Transform(tf::createIdentityQuaternion(),
                                                          tf::Vector3(armor_msg.pose.x, armor_msg.pose.y, armor_msg.pose.z)),
                                            ros::Time(), "usb_camera_link");
                tf::Stamped<tf::Pose> pose;

                try
                {

                    tf_->transformPose("odom", ident, pose);
                }
                catch (tf::TransformException &e)
                {
                    ROS_ERROR("Couldn't transform "
                              "even though the message notifier is in use");
                    break;
                }
                ROS_INFO("%f %f", pose.getOrigin().x(), pose.getOrigin().y());
                armor_msg.pose_odom.x = pose.getOrigin().x();
                armor_msg.pose_odom.y = pose.getOrigin().y();
                armor_msg.distance =
                    sqrt(pow(enemy_pose.position.x - armor_msg.pose_odom.x, 2) +
                         pow(enemy_pose.position.y - armor_msg.pose_odom.y, 2));
                msg_armor_info.armor_list.push_back(armor_msg);

                if (armor_msg.distance < distance)
                {
                    idx = i;
                    distance = armor_msg.distance;
                }
            }
            if (idx == -1)
                continue;

            // the Fittest Armor
            float pre_cur_dis;
            if (pre_enemy_pose.x == 0)
            {
                pre_cur_dis = 0;
            }
            else
            {
                pre_cur_dis = sqrt(pow(msg_armor_info.armor_list[idx].pose.x -
                                           pre_enemy_pose.x,
                                       2) +
                                   pow(msg_armor_info.armor_list[idx].pose.y -
                                           pre_enemy_pose.y,
                                       2));
            }
            if (1)//distance < 0.5 && pre_cur_dis < 0.5
            {
                msg_armor_info.pose_camera =
                    msg_armor_info.armor_list[idx].pose;
                msg_armor_info.pose_odom =
                    msg_armor_info.armor_list[idx].pose_odom;
                msg_armor_info.angle = msg_armor_info.armor_list[idx].angle;
                msg_armor_info.pose_image.x = armor_target[idx].center.x;
                msg_armor_info.pose_image.y = armor_target[idx].center.y;
                if (abs(msg_armor_info.angle.x) < 10 &&
                    abs(msg_armor_info.angle.y) < 10)
                {
                    msg_armor_info.mode = 3; //对准
                }
                else
                {
                    msg_armor_info.mode = 2; //没有对准
                }
                if (msg_armor_info.angle.x > 2000)
                {
                    msg_armor_info.angle.x = 2000;
                }
                if (msg_armor_info.angle.x < -2000)
                {
                    msg_armor_info.angle.x = -2000;
                }
            }

            pub_armor_info.publish(msg_armor_info);
        }
        else
        {
            // lost
            miss_detection_cnt++;
            if (miss_detection_cnt < 5)
            {
                // msg_armor_info.angle.x = (pre_angle_x + offset_anlge_x) *
                // 100;  // yaw
                msg_armor_info.angle.y = pre_angle.y;
            }
            else
            {
                pre_enemy_pose.x = 0;
                pre_enemy_pose.y = 0;
            }
            msg_armor_info.mode = 1; //没检测到目标
            pub_armor_info.publish(msg_armor_info);
        }

        if (setting.show_image > 0)
        {
            circle(src_csm, image_center, 3, CV_RGB(0, 255, 0), 2);
            for (int i = 0; i < armor_target.size(); i++)
            {
                circle(src_csm, armor_target[i].ld, 2, CV_RGB(255, 255, 0), 2);
                circle(src_csm, armor_target[i].lu, 2, CV_RGB(255, 255, 0), 2);
                circle(src_csm, armor_target[i].ru, 2, CV_RGB(255, 255, 0), 2);
                circle(src_csm, armor_target[i].rd, 2, CV_RGB(255, 255, 0), 2);
                line(src_csm, armor_target[i].ld, armor_target[i].lu,
                     CV_RGB(0, 255, 0), 1);
                line(src_csm, armor_target[i].lu, armor_target[i].ru,
                     CV_RGB(0, 255, 0), 1);
                line(src_csm, armor_target[i].ru, armor_target[i].rd,
                     CV_RGB(0, 255, 0), 1);
                line(src_csm, armor_target[i].rd, armor_target[i].ld,
                     CV_RGB(0, 255, 0), 1);
            }
            circle(
                src_csm,
                Point(msg_armor_info.pose_image.x, msg_armor_info.pose_image.y),
                3, CV_RGB(0, 0, 255), 3);

            char str[30];
            sprintf(str, "angle %.1f, %.1f", msg_armor_info.angle.x,
                    msg_armor_info.angle.y);
            putText(src_csm, str, Point(10, 40), CV_FONT_HERSHEY_COMPLEX_SMALL,
                    1.3, CV_RGB(128, 255, 0), 1);
            char str2[30];
            sprintf(str2, "pose in cam %.3f, %.3f, %.3f", msg_armor_info.pose_camera.x,
                    msg_armor_info.pose_camera.y, msg_armor_info.pose_camera.z);
            putText(src_csm, str2, Point(10, 80), CV_FONT_HERSHEY_COMPLEX_SMALL,
                    1, CV_RGB(128, 255, 0), 1);
            Mat s = angle_solver.position_in_ptz;
            sprintf(str2, "pose in odom %.3f, %.3f, %.3f", msg_armor_info.pose_odom.x,
                    msg_armor_info.pose_odom.y, msg_armor_info.pose_odom.z);
            putText(src_csm, str2, Point(10, 120),
                    CV_FONT_HERSHEY_COMPLEX_SMALL, 1, CV_RGB(128, 255, 0), 1);
            writer << src_csm;
            imshow("result", src_csm);
            waitKey(1);
        }
        ros::Time end = ros::Time::now();
        ROS_INFO("Time %f", (end - start).toSec());
    }

    ros::spinOnce();
    rate.sleep();
}
