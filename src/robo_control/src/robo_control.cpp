#include "robo_control/robo_control.hpp"
void RoboControl::cb_armorInfo(const robo_vision::ArmorInfo &msg)
{
    armor_info_msg.mode = msg.mode;
    armor_info_msg.image_dx = msg.pose_image.x;
    armor_info_msg.image_dy = msg.pose_image.y;
    armor_info_msg.global_z = msg.pose_global.position.z;
    armor_info_msg.pitch = msg.angle.y;
    armor_info_msg.yaw = msg.angle.x;
    armor_ready_flag = 1;
}

void RoboControl::cb_cmd_vel(const geometry_msgs::Twist &msg)
{
    cmd_vel_msg.v_x = msg.linear.x;
    cmd_vel_msg.v_y = msg.linear.y;
    cmd_vel_msg.v_yaw = msg.angular.z;
}

void RoboControl::cb_move_base(const move_base_msgs::MoveBaseActionFeedback &msg)
{
    nav_status = msg.status.status;
    if (nav_status != 1)
    {
        nav_current_goal.position.x = 0;
        nav_current_goal.position.y = 0;
        nav_current_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    }
}




void RoboControl::cb_enemy_pose(const geometry_msgs::TransformStamped &msg)
{
    enemy_odom_pose.position.x = msg.transform.translation.x;
    enemy_odom_pose.position.y = msg.transform.translation.y;
    enemy_odom_pose.orientation = robo_ukf_pose.orientation;
    enemy_found_flag = 1;
}

void RoboControl::cb_enemy_pnp_pose(const geometry_msgs::TransformStamped &msg)
{
    enemy_odom_pnp_pose.position.x = msg.transform.translation.x - 0.2;
    enemy_odom_pnp_pose.position.y = msg.transform.translation.y - 0.2;
    enemy_odom_pnp_pose.orientation = robo_ukf_pose.orientation;
    enemy_found_pnp_flag = 1;
}

void RoboControl::cb_ukf_pose(const nav_msgs::Odometry &msg)
{
    robo_ukf_pose = msg.pose.pose;
}

void RoboControl::cb_another_robo_odom(const nav_msgs::Odometry &msg)
{
    another_robo_pose = msg.pose.pose;
}

void RoboControl::cb_move_base_status(const actionlib_msgs::GoalStatusArray &msg)
{
    ROS_INFO("receive!!!!!!!!!!!!!!!!!!!");
}
void RoboControl::cb_cur_goal(const geometry_msgs::PoseStamped &msg)
{
    nav_current_goal = msg.pose;
    nav_current_goal.position.x += 1;
    nav_current_goal.position.y += 1;
}

void RoboControl::readMCUData()
{
    ros::Time time = ros::Time::now();
    RobotMsgFromMCU msg_frommcu;
    if (!serial->ReadData(msg_frommcu))
        return;

    init_flag = msg_frommcu.init_flag;
    robo_control::GameInfo game_msg;
    game_msg.header.stamp = time;
    game_msg.header.frame_id = "base_link";
    game_msg.remainingHP = msg_frommcu.remaining_HP;
    game_msg.attackArmorID = msg_frommcu.attack_armorID;
    game_msg.bulletCount = msg_frommcu.remaining_bullet;
    game_msg.bulletCount = msg_frommcu.uwb_yaw;
    game_msg.gimbalAngleYaw = msg_frommcu.gimbal_chassis_angle * 1.0 / 100;
    game_msg.gimbalAnglePitch = -msg_frommcu.gimbal_pitch_angle * 1.0 / 100;
    pub_game_info.publish(game_msg);

    nav_msgs::Odometry uwb_odom_msg;
    uwb_odom_msg.header.stamp = time;
    uwb_odom_msg.header.frame_id = "odom";
    uwb_odom_msg.child_frame_id = "base_link";
    uwb_odom_msg.pose.pose.position.x = msg_frommcu.uwb_x * 1.0 / 100;
    uwb_odom_msg.pose.pose.position.y = msg_frommcu.uwb_y * 1.0 / 100;
    uwb_odom_msg.pose.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(
            0, 0, (msg_frommcu.uwb_yaw * 1.0 / 100) * PI * 2 / 360);
    //     uwb_odom_msg.twist.twist.linear.x = msg_frommcu.wheel_odom_x
    //     * 1.0 / 10000; uwb_odom_msg.twist.twist.linear.y =
    //     msg_frommcu.wheel_odom_y * 1.0 / 10000;
    robo_uwb_pose = uwb_odom_msg.pose.pose;
    if (pre_uwb_ready_flag != msg_frommcu.uwb_ready_flag)
    {
        pub_uwb_odom.publish(uwb_odom_msg);
    }
    pre_uwb_ready_flag = msg_frommcu.uwb_ready_flag;

    geometry_msgs::Vector3Stamped wheel_vel_msg;
    wheel_vel_msg.header.stamp = time;
    wheel_vel_msg.header.frame_id = "base_link";
    wheel_vel_msg.vector.x = msg_frommcu.wheel_odom_x * 1.0 / 10000;
    wheel_vel_msg.vector.y = msg_frommcu.wheel_odom_y * 1.0 / 10000;
    pub_wheel_vel.publish(wheel_vel_msg);

    geometry_msgs::TransformStamped gimbal_trans;
    gimbal_trans.header.stamp = time;
    gimbal_trans.header.frame_id = "base_link";
    gimbal_trans.child_frame_id = "gimbal_link";
    gimbal_trans.transform.translation.x = 0.15;
    gimbal_trans.transform.translation.y = 0;
    gimbal_trans.transform.translation.z = 0.0;
    gimbal_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(
        0, -msg_frommcu.gimbal_pitch_angle * 1.0 / 100 * PI * 2 / 360,
        msg_frommcu.gimbal_chassis_angle * 1.0 / 100 * PI * 2 / 360);
    gimbal_tf.sendTransform(gimbal_trans);

    geometry_msgs::TransformStamped camera_trans;
    camera_trans.header.stamp = time;
    camera_trans.header.frame_id = "gimbal_link";
    camera_trans.child_frame_id = "usb_camera_link";
    camera_trans.transform.translation.x = 0.15;
    camera_trans.transform.translation.y = 0;
    camera_trans.transform.translation.z = 0.0;
    camera_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(-1.57, 0, -1.57);
    camera_tf.sendTransform(camera_trans);
}

void RoboControl::sendMCUMsg(int _chassis_mode, int _gimbal_mode,
                             float _velocity_x, float _velocity_y,
                             float _velocity_yaw, float _gimbal_yaw,
                             float _gimbal_pitch, float _enemy_distance)
{
    struct RobotMsgToMCU msg_tomcu;
    msg_tomcu.chassis_mode = _chassis_mode;
    msg_tomcu.gimbal_mode = _gimbal_mode;
    msg_tomcu.velocity_x = _velocity_x * 100;
    msg_tomcu.velocity_y = _velocity_y * 100;
    msg_tomcu.velocity_yaw = _velocity_yaw * KYAW;
    msg_tomcu.gimbal_yaw = _gimbal_yaw;
    msg_tomcu.gimbal_pitch = _gimbal_pitch;
    msg_tomcu.enemy_distance = _enemy_distance;
    serial->SendData(msg_tomcu);
    armor_ready_flag = 0;
}

void RoboControl::sendNavGoal(geometry_msgs::Pose &target_pose)
{
    ROS_INFO("tar: %f  %f  %f  %f", target_pose.position.x,
             target_pose.position.y, nav_current_goal.position.x,
             nav_current_goal.position.y);
    //     if (abs(target_pose.position.x - nav_current_goal.position.x) > 0.2 ||
    //         abs(target_pose.position.y - nav_current_goal.position.y) > 0.2) {
    pub_nav_goal.publish(target_pose);
    //     }
}

bool RoboControl::judgeKeyPointPosition(KeyPoint currentPosition,
                                        KeyPoint goalKeyPoint,
                                        float x_threshold, float y_threshold,
                                        float yaw_threshold)
{
    float x_error = abs(currentPosition.x - goalKeyPoint.x);
    float y_error = abs(currentPosition.y - goalKeyPoint.y);
    // float yaw_error = abs(currentPosition.yaw - goalKeyPoint.yaw);
    float yaw_error = 0;
    if (x_error < x_threshold && y_error < y_threshold &&
        yaw_error < yaw_threshold)
    {
        return (true);
    }
    return (false);
}

bool RoboControl::keyPointNav(int keypoint_num)
{
    KeyPoint goalKeyPoint = KEY_POINT[keypoint_num];
    KeyPoint currentPosition = KEY_POINT[keypoint_num];
    currentPosition.x = robo_ukf_pose.position.x;
    currentPosition.y = robo_ukf_pose.position.y;

    geometry_msgs::Pose nav_goal;
    nav_goal.position.x = goalKeyPoint.x;
    nav_goal.position.y = goalKeyPoint.y;

    float angle = 0.0;
    if (goalKeyPoint.yaw != 0.0 || goalKeyPoint.yaw != 180 * PI / 180.0)
    {
        angle = 0.0;
    }
    nav_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    sendNavGoal(nav_goal);

    float x_threshold = 0.5;
    float y_threshold = 0.5;
    float yaw_threshold = 0.2;

    float x_error = abs(currentPosition.x - goalKeyPoint.x);
    float y_error = abs(currentPosition.y - goalKeyPoint.y);
    float yaw_error = 0;
    // float yaw_error = abs(currentPosition.yaw - goalKeyPoint.yaw);
    ROS_INFO("Goal: %d %f %f %f, error: %f %f %f", keypoint_num, goalKeyPoint.x,
             goalKeyPoint.y, goalKeyPoint.yaw, x_error, y_error, yaw_error);

    if (judgeKeyPointPosition(currentPosition, goalKeyPoint, x_threshold,
                              y_threshold, yaw_threshold))
    {
        ROS_INFO("robo_ctl.nav_status");
        nav_start_time = ros::Time::now();
        return true;
    }
    ros::Duration dtime = ros::Time::now() - nav_start_time;
    if (dtime.toSec() > 9)
    {
        nav_start_time = ros::Time::now();
        return true;
    }
    return false;
}

int RoboControl::getClosestKeyPoint(float pose_x, float pose_y)
{
    float distance = 1000;
    int num = 0;
    for (int i = 0; i < 23; i++)
    {
        float cur_dis = (pose_x - KEY_POINT[i].x) * (pose_x - KEY_POINT[i].x) +
                        (pose_y - KEY_POINT[i].y) * (pose_y - KEY_POINT[i].y);
        if (cur_dis < distance)
        {
            num = i;
            distance = cur_dis;
        }
    }
    return num;
}

void RoboControl::cb_finish_navigation(const std_msgs::Bool &msg)
{
    finish_navigation = msg;
}

void RoboControl::go_on_patrol(int flag, float current_position, float enemy_position)
{
    /*************************************************************************
    *  函数名称：go_on_patrol
    *  功能说明：巡图
    *  参数说明：   flag: 1 -> 从中点开始的巡图, 2 -> 丢失敌人巡图
    *               current_position: 自身当前位置
    *               enemy_position: 敌人最后一次出现的位置
    *  函数返回：无
    *************************************************************************/
   if(flag == 1)
   {
       // 在中点处的巡图
       
   }
   if(flag == 2)
   {
       // 丢失敌人之后的巡图
   }
}