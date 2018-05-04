#pragma once
#include <string>
#include <cmath>
#include <time.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>

#include <actionlib_msgs/GoalStatusArray.h>

#include <opencv2/opencv.hpp>

#include "robo_control/serial.h"
#include "robo_vision/ArmorInfo.h"
#include "robo_control/GameInfo.h"
#include "robo_perception/ObjectList.h"

using namespace std;
#define PI 3.1415926535898
#define KYAW 70

//Armor Info Data
struct ArmorInfo
{
    int mode;
    float image_dx;
    float image_dy;
    float global_z;
    float pitch;
    float yaw;
};

//Chassis Velocity Data
struct ChassisCMD
{
    float v_x = 0;
    float v_y = 0;
    float v_yaw = 0;
};

struct KeyPoint
{
    int idx;   // index of KeyPoint
    float x;   // x coordinate of keypoint
    float y;   // y coordinate of keypoint
    float yaw; // the direction of keypoint
};

class RoboControl
{
  public:
    RoboControl()
    {
        pnh = new ros::NodeHandle("");

        pub_game_info = pnh->advertise<robo_control::GameInfo>("base/game_info", 1);
        pub_uwb_odom = pnh->advertise<nav_msgs::Odometry>("map/uwb/data", 1);
        pub_wheel_vel = pnh->advertise<geometry_msgs::Vector3Stamped>("robo/wheel/data", 1);
        pub_imu_data = pnh->advertise<sensor_msgs::Imu>("gimbal/imu/data_raw", 1);
        pub_nav_goal = pnh->advertise<geometry_msgs::Pose>("base/goal", 1);
        pub_enemy_target = pnh->advertise<nav_msgs::Odometry>("enemy/target", 1);

        serial = new Serial("/dev/ttyUSBA1");
        serial->configurePort();

        pre_uwb_ready_flag = 0;
        enemy_found_flag = 0;
        armor_ready_flag = 0;
        init_flag = 0;
        nav_start_time = ros::Time::now();
    };

    //ROS CallBack Function
    void cb_armorInfo(const robo_vision::ArmorInfo &msg);
    void cb_cmd_vel(const geometry_msgs::Twist &msg);
    void cb_move_base(const move_base_msgs::MoveBaseActionFeedback &msg);
    void cb_enemy_pose(const geometry_msgs::TransformStamped &msg);
    void cb_enemy_pnp_pose(const geometry_msgs::TransformStamped &msg);
    void cb_ukf_pose(const nav_msgs::Odometry &msg);
    void cb_cur_goal(const geometry_msgs::PoseStamped &msg);
    void cb_move_base_status(const actionlib_msgs::GoalStatusArray &msg);
    void cb_another_robo_odom(const nav_msgs::Odometry &msg);

    void readMCUData();
    void sendNavGoal(geometry_msgs::Pose &target_pose);
    void sendMCUMsg(int _chassis_mode, int _gimbal_mode, float _velocity_x, float _velocity_y,
                    float _velocity_yaw, float _gimbal_yaw, float _gimbal_pitch,
                    float _enemy_distance);
    bool judgeKeyPointPosition(KeyPoint currentPosition,
                               KeyPoint goalKeyPoint,
                               float x_threshold, float y_threshold, float yaw_threshold);
    int getClosestKeyPoint(float pose_x, float pose_y);

    void cb_finish_navigation(const std_msgs::Bool &msg);
    void go_on_patrol(int flag, int key_point_count, float current_position, float enemy_position);
    void cb_enemy_information(const robo_perception::ObjectList &msg);
    void read_xml_file();
    int find_enemy_self_closest_point(double enemy_x, double enemy_y, double self_x, double self_y);
    void sendEnemyTarget(const geometry_msgs::Pose &msg);
    ros::NodeHandle *pnh;

    ros::Publisher pub_game_info;
    ros::Publisher pub_uwb_odom;
    ros::Publisher pub_wheel_vel;
    ros::Publisher pub_imu_data;
    ros::Publisher pub_nav_goal;
    ros::Publisher pub_enemy_target;

    tf2_ros::TransformBroadcaster gimbal_tf;
    tf2_ros::TransformBroadcaster camera_tf;

    Serial *serial;

    int uwb_ready_flag; //current UWB flag  valid if = 2
    int pre_uwb_ready_flag;

    int enemy_found_flag;     //if enemy found in realsence -> 1
    int enemy_found_pnp_flag; //if enemy found in pnp -> 1
    int armor_ready_flag;     //armor data come ->1  not = find armor
    int init_flag;
    ArmorInfo armor_info_msg;

    ChassisCMD cmd_vel_msg;

    geometry_msgs::Pose robo_uwb_pose;
    geometry_msgs::Pose robo_ukf_pose;
    geometry_msgs::Pose enemy_odom_pose;
    geometry_msgs::Pose enemy_odom_pnp_pose;
    geometry_msgs::Pose another_robo_pose;

    /* uint8 PENDING=0
     *  uint8 ACTIVE=1
     *  uint8 PREEMPTED=2
     *  uint8 SUCCEEDED=3
     *  uint8 ABORTED=4
     *  uint8 REJECTED=5
     *  uint8 PREEMPTING=6
     *  uint8 RECALLING=7
     *  uint8 RECALLED=8
     *  uint8 LOST=9*/
    int nav_status; //status from move_base pkg

    geometry_msgs::Pose nav_current_goal;
    
    bool keyPointNav(int keypoint_num);
    ros::Time nav_start_time;

    std_msgs::Bool finish_navigation;

    robo_perception::ObjectList enemy_information;

    // 各种 flag
    bool finish_goto_center = false;

    int key_point_count = 1;

    cv::Mat point_list;

    // yaw: 1 -> 2
    KeyPoint KEY_POINT[30] = {
        {0, 1.30, 0.80, 0 * PI / 180.0},
        {1, 2.60, 0.80, 90 * PI / 180.0},
        {2, 2.60, 1.50, 90 * PI / 180.0},
        {3, 2.60, 2.50, 90 * PI / 180.0},

        //{4, 2.00, 3.20, 0*PI/180.0},
        //{5, 3.00, 3.50, 0*PI/180.0},

        {6, 4.00, 2.50, -90 * PI / 180.0},
        {7, 4.00, 1.50, -90 * PI / 180.0},
        {8, 4.00, 0.50, 0 * PI / 180.0},
        {9, 5.00, 0.50, 0 * PI / 180.0},
        {10, 6.40, 0.50, 0 * PI / 180.0},
        {11, 7.50, 1.0, 90 * PI / 180.0},
        //{12, 7.10, 1.80, 90*PI/180.0},
        //{13, 6.70, 2.50, 90*PI/180.0},
        {14, 6.70, 3.50, 90 * PI / 180.0},
        {15, 6.70, 4.20, 180 * PI / 180.0},
        {16, 5.40, 4.20, -90 * PI / 180.0},
        {17, 5.40, 3.50, -90 * PI / 180.0},
        {18, 5.40, 2.50, -90 * PI / 180.0},

        //{19, 6.00, 1.80, 180*PI/180.0},
        //{20, 5.00, 1.50, 180*PI/180.0},

        {21, 4.00, 2.50, 90 * PI / 180.0},
        {22, 4.00, 3.50, 90 * PI / 180.0},
        {23, 3.80, 4.20, 180 * PI / 180.0},
        {24, 3.00, 4.50, 180 * PI / 180.0},
        {25, 1.60, 4.50, 180 * PI / 180.0},
        {26, 0.60, 4.50, -90 * PI / 180.0},
        //{27, 0.90, 3.20, -90*PI/180.0},
        {28, 1.30, 2.50, -90 * PI / 180.0},
        {29, 1.30, 1.50, -90 * PI / 180.0},
    };
};
