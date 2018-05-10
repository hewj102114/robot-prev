#include "robo_control/robo_control.hpp"
void RoboControl::cb_armorInfo(const robo_vision::ArmorInfo &msg)
{
    armor_info_msg.mode = msg.mode;
    armor_info_msg.image_dx = msg.pose_image.x;
    armor_info_msg.image_dy = msg.pose_image.y;
    armor_info_msg.global_z = msg.pose_camera.z;
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
    
    game_msg.header.stamp = time;
    game_msg.header.frame_id = "base_link";
    game_msg.remainingHP = msg_frommcu.remaining_HP;
    game_msg.attackArmorID = msg_frommcu.attack_armorID;
    game_msg.bulletCount = msg_frommcu.remaining_bullet;
    game_msg.bulletCount = msg_frommcu.uwb_yaw;
    game_msg.gimbalAngleYaw = msg_frommcu.gimbal_chassis_angle * 1.0 / 100;
    game_msg.gimbalAnglePitch = -msg_frommcu.gimbal_pitch_angle * 1.0 / 100;
    game_msg.bulletSpeed = msg_frommcu.bullet_speed*1.0/1000;
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
    callback_navigation_flag = false;
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
    callback_navigation_flag = true;
}

void RoboControl::cb_enemy_information(const robo_perception::ObjectList &msg)
{
    enemy_information = msg;
}

void RoboControl::go_on_patrol(int flag, int key_point_count, float current_position, float enemy_position)
{
    /*************************************************************************
    *  函数名称：go_on_patrol
    *  功能说明：巡图
    *  参数说明：   flag: 1 -> 从中点开始的巡图, 2 -> 丢失敌人巡图
    *               current_position: 自身当前位置
    *               enemy_position: 敌人最后一次出现的位置
    *  函数返回：返回下次去的位置
    *************************************************************************/
   geometry_msgs::Pose target_pose;
   if(flag == 1)
   {
       // 从中点开始的巡图
       float x_go_on_patrol[4] = {1.30, 6.70, 6.70, 1.30};
       float y_go_on_patrol[4] = {0.80, 0.80, 4.20, 4.20};
       
       target_pose.position.x = x_go_on_patrol[key_point_count];
       target_pose.position.y = y_go_on_patrol[key_point_count];
       target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
       sendNavGoal(target_pose);
   }
   if(flag == 2)
   {
       // 丢失敌人之后的巡图
   }
}

void RoboControl::read_xml_file()
{
    cv::FileStorage fs("/home/ubuntu/robot/src/robo_navigation/launch/matrix.xml", cv::FileStorage::READ);
    fs["Point"] >> point_list;
} 

int RoboControl::find_enemy_self_closest_point(double enemy_x, double enemy_y, double self_x, double self_y)
{
    vector<float> dis_list;
    for (int i = 0; i < point_list.rows; i++)
    {
        float d_enemy_x = enemy_x - point_list.at<double>(i, 0) * 1.0 / 100;
        float d_enemy_y = enemy_y - point_list.at<double>(i, 1) * 1.0 / 100;

        float d_self_x = self_x - point_list.at<double>(i, 0) * 1.0 / 100;
        float d_self_y = self_y - point_list.at<double>(i, 1) * 1.0 / 100;
        
        float distance_enemy = sqrt(d_enemy_x * d_enemy_x + d_enemy_y * d_enemy_y);
        float distance_self = sqrt(d_self_x * d_self_x + d_self_y * d_self_y);
        
        if (distance_enemy < 1.0 || distance_enemy > 2.0)
        {
            distance_enemy = 1000.0;
        }

        dis_list.push_back(distance_self + distance_enemy);
    }

    vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end());

    int n = distance(dis_list.begin(), smallest);
    return n;
}

geometry_msgs::Pose RoboControl::sendEnemyTarget(const robo_perception::ObjectList &msg, geometry_msgs::Pose &last_enemy_target_msg)
{
    geometry_msgs::Pose result_enemy_target;
    int enemy_index1 = 0;
    int enemy_index2 = 0;

    nav_msgs::Odometry enemy_odom_target_msg;
    enemy_odom_target_msg.header.stamp = ros::Time::now();
    enemy_odom_target_msg.header.frame_id = "odom";
    enemy_odom_target_msg.child_frame_id = "target";

    // 计算打击哪个车, 给定 infrared detection 的检测结果和上一帧的打击目标
    if (msg.red_num == 0)
    {
        // 丢失敌人
        result_enemy_target.position.x = 0;
        result_enemy_target.position.y = 0;
        result_enemy_target.position.z = 0;
        return result_enemy_target; 
    }
    if (msg.red_num == 1)
    {
        // 没有选择, 只打当前的敌人
        for (int i = 0; i < msg.num; i++)
        {
            if(msg.object[i].team.data == "red0")
            {
                enemy_index1 = i;
            }
        }

        enemy_odom_target_msg.pose.pose.position.x = msg.object[enemy_index1].globalpose.position.x;
        enemy_odom_target_msg.pose.pose.position.y = msg.object[enemy_index1].globalpose.position.y;
        enemy_odom_target_msg.pose.pose.orientation = msg.object[enemy_index1].globalpose.orientation;
        enemy_odom_target_msg.pose.pose.orientation.w = 1;
        result_enemy_target = enemy_odom_target_msg.pose.pose;
    }
    if (msg.red_num == 2)
    {
        // 判断之前有没有打击过敌人, 如果之前没有打击过敌人, 选择距离近的敌人, 否则选择之前打击过的
        for (int i = 0; i < msg.num; i++)
        {
            if (msg.object[i].team.data == "red0")
            {
                enemy_index1 = i;
            }
            if (msg.object[i].team.data == "red1")
            {
                enemy_index2 = i;
            }
        }
        if (last_enemy_target_msg.position.x == 0 && last_enemy_target_msg.position.y == 0 && last_enemy_target_msg.position.z == 0)
        {
            // 没有打击过敌人, 选择相对距离近的敌人
            if (msg.object[enemy_index1].pose.position.x < msg.object[enemy_index2].pose.position.x)
            {
                // 选择第一个敌人
                enemy_odom_target_msg.pose.pose.position.x = msg.object[enemy_index1].globalpose.position.x;
                enemy_odom_target_msg.pose.pose.position.y = msg.object[enemy_index1].globalpose.position.y;
                enemy_odom_target_msg.pose.pose.orientation = msg.object[enemy_index1].globalpose.orientation;
                enemy_odom_target_msg.pose.pose.orientation.w = 1;
                result_enemy_target = enemy_odom_target_msg.pose.pose;
            }
            else
            {
                // 选择第二个敌人
                enemy_odom_target_msg.pose.pose.position.x = msg.object[enemy_index2].globalpose.position.x;
                enemy_odom_target_msg.pose.pose.position.y = msg.object[enemy_index2].globalpose.position.y;
                enemy_odom_target_msg.pose.pose.orientation = msg.object[enemy_index2].globalpose.orientation;
                enemy_odom_target_msg.pose.pose.orientation.w = 1;
                result_enemy_target = enemy_odom_target_msg.pose.pose;
            }
        }   
        else
        {
            // 之前打击过敌人, 选择离之前的选择近的敌人
            for (int i = 0; i < 2; i++)
            {
                float dis1 = pow(msg.object[enemy_index1].globalpose.position.x - last_enemy_target_msg.position.x, 2) + pow(msg.object[enemy_index1].globalpose.position.y - last_enemy_target_msg.position.y, 2);
                float dis2 = pow(msg.object[enemy_index2].globalpose.position.x - last_enemy_target_msg.position.x, 2) + pow(msg.object[enemy_index2].globalpose.position.y - last_enemy_target_msg.position.y, 2);
                if (dis1 <= dis2)
                {
                    enemy_odom_target_msg.pose.pose.position.x = msg.object[enemy_index1].globalpose.position.x;
                    enemy_odom_target_msg.pose.pose.position.y = msg.object[enemy_index1].globalpose.position.y;
                    enemy_odom_target_msg.pose.pose.orientation = msg.object[enemy_index1].globalpose.orientation;
                    enemy_odom_target_msg.pose.pose.orientation.w = 1;
                    result_enemy_target = enemy_odom_target_msg.pose.pose;
                }
                else
                {
                    enemy_odom_target_msg.pose.pose.position.x = msg.object[enemy_index2].globalpose.position.x;
                    enemy_odom_target_msg.pose.pose.position.y = msg.object[enemy_index2].globalpose.position.y;
                    enemy_odom_target_msg.pose.pose.orientation = msg.object[enemy_index2].globalpose.orientation;
                    enemy_odom_target_msg.pose.pose.orientation.w = 1;
                    result_enemy_target = enemy_odom_target_msg.pose.pose;
                }
            }
        }
    }
    pub_enemy_target.publish(enemy_odom_target_msg);
    return result_enemy_target;
}

float RoboControl::calculator_enemy_angle(double enemy_x, double enemy_y, double self_x, double self_y)
{
    float dx = enemy_x - self_x;
    float dy = enemy_y - self_y;
    float angle = atan2(dy, dx);
    return angle * 180.0 / PI + 90.0;
}

void RoboControl::cb_ukf_enemy_information(const nav_msgs::Odometry &msg)
{
    robo_ukf_enemy_information = msg.pose.pose;
}
