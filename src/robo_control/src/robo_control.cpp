#include "robo_control/robo_control.hpp"
void RoboControl::cb_armorInfo(const robo_vision::ArmorInfo &msg)
{
    armor_info_msg.mode = msg.mode;
    armor_info_msg.global_z = msg.target.pose_base.x;
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

void RoboControl::cb_front_dis(const std_msgs::Float64 &msg)
{
    front_dis = msg.data;
}

void RoboControl::main_control_init()
{
    sent_mcu_gimbal_msg.mode = 1;
    sent_mcu_gimbal_msg.yaw = 0;
    sent_mcu_gimbal_msg.pitch = 0;
    sent_mcu_gimbal_msg.global_z = 0;

    sent_mcu_vel_result.mode = 1;
    sent_mcu_vel_result.v_x = 0;
    sent_mcu_vel_result.v_y = 0;
    sent_mcu_vel_result.v_yaw = 0;

    enemy_information.header.frame_id = "enemy";
    enemy_information.header.stamp = ros::Time::now();
    enemy_information.num = 0;
    enemy_information.red_num = 0;
    enemy_information.death_num = 0;
    enemy_information.blue_num = 0;

    robo_perception::Object temp_object;
    temp_object.team.data = "Nothing";
    temp_object.pose.position.x = 0;
    temp_object.pose.position.y = 0;
    temp_object.pose.position.z = 0;

    temp_object.globalpose.position.x = 0;
    temp_object.globalpose.position.y = 0;
    temp_object.globalpose.position.z = 0;

    enemy_information.object.push_back(temp_object);

    last_enemy_target_pose.position.x = 0;
    last_enemy_target_pose.position.y = 0;
    last_enemy_target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
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
    game_msg.gimbalAngleYaw = msg_frommcu.gimbal_chassis_angle * 1.0 / 100;
    game_msg.gimbalAnglePitch = -msg_frommcu.gimbal_pitch_angle * 1.0 / 100;
    game_msg.bulletSpeed = msg_frommcu.bullet_speed * 1.0 / 1000;
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
    gimbal_trans.transform.translation.y = 0.0;
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
    if (flag == 1)
    {
        // 从中点开始的巡图
        float x_go_on_patrol[4] = {1.30, 6.70, 6.70, 1.30};
        float y_go_on_patrol[4] = {0.80, 0.80, 4.20, 4.20};

        target_pose.position.x = x_go_on_patrol[key_point_count];
        target_pose.position.y = y_go_on_patrol[key_point_count];
        target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
        sendNavGoal(target_pose);
    }
    if (flag == 2)
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
        ROS_INFO("find_enemy_self_closest_point: %d", i);
        float d_enemy_y = enemy_y - point_list.at<double>(i, 0) * 1.0 / 100;
        float d_enemy_x = enemy_x - point_list.at<double>(i, 1) * 1.0 / 100;

        float d_self_y = self_y - point_list.at<double>(i, 0) * 1.0 / 100;
        float d_self_x = self_x - point_list.at<double>(i, 1) * 1.0 / 100;

        float distance_enemy = sqrt(0.4 * d_enemy_x * d_enemy_x + 0.6 * d_enemy_y * d_enemy_y);
        float distance_self = sqrt(0.4 * d_self_x * d_self_x + 0.6 * d_self_y * d_self_y);

        if (distance_enemy < 0.5 || distance_enemy > 2.0)
        {
            distance_enemy = 1000.0;
        }

        dis_list.push_back(0.4 * distance_self + 0.6 * distance_enemy);
    }

    vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end());

    int n = distance(dis_list.begin(), smallest);
    return n;
}

robo_perception::ObjectList RoboControl::sendEnemyTarget(const robo_perception::ObjectList &msg, robo_perception::ObjectList &last_enemy_target_msg)
{
    ROS_INFO("OK11");
    vector<int> enemy_index;
    vector<float> enemy_self_distance; // 当前敌人与自己的相对距离
    vector<float> enemy_last_distance; // 当前敌人与上一帧敌人的距离

    robo_perception::ObjectList result_enemy_target; // return
    result_enemy_target.header = msg.header;
    result_enemy_target.header.stamp = ros::Time::now();
    ROS_INFO("OK12");

    result_enemy_target.num = 0;
    result_enemy_target.red_num = 0;
    result_enemy_target.death_num = 0;
    result_enemy_target.blue_num = 0;
    int enemy_index1 = 0;
    int enemy_index2 = 0;
    ROS_INFO("OK13");

    robo_perception::ObjectList enemy_odom_target_msg; // publish
    enemy_odom_target_msg.header = msg.header;
    result_enemy_target.header.stamp = ros::Time::now();
    ROS_INFO("OK14");

    enemy_odom_target_msg.num = 0;
    enemy_odom_target_msg.red_num = 0;
    enemy_odom_target_msg.death_num = 0;
    enemy_odom_target_msg.blue_num = 0;

    robo_perception::Object temp_object;
    ROS_INFO("OK15");
    ROS_INFO("red_num:", msg.red_num);

    // 计算打击哪个车, 给定 infrared detection 的检测结果和上一帧的打击目标
    if (msg.red_num == 0)
    {
        // 丢失敌人
        ROS_INFO("red_num = 0");
        temp_object.team.data = "Nothing";
        temp_object.pose.position.x = 0;
        temp_object.pose.position.y = 0;
        temp_object.pose.position.z = 0;

        temp_object.globalpose.position.x = 0;
        temp_object.globalpose.position.y = 0;
        temp_object.globalpose.position.z = 0;
        ROS_INFO("OK16");

        result_enemy_target.object.push_back(temp_object);
        enemy_odom_target_msg = result_enemy_target;
        pub_enemy_target.publish(enemy_odom_target_msg);
        return result_enemy_target;
    }
    if (msg.red_num == 1)
    {
        ROS_INFO("red_num = 1");

        // 没有选择, 只打当前的敌人
        for (int i = 0; i < msg.num; i++)
        {
            if (msg.object[i].team.data == "red0")
            {
                enemy_index1 = i;
            }
        }
        ROS_INFO("OK17");

        enemy_odom_target_msg.num = 1;
        enemy_odom_target_msg.red_num = 1;
        enemy_odom_target_msg.object.push_back(msg.object[enemy_index1]);
        enemy_odom_target_msg.object[0].pose.orientation.w = 1;
        enemy_odom_target_msg.object[0].globalpose.orientation.w = 1;
        result_enemy_target = enemy_odom_target_msg;
        pub_enemy_target.publish(enemy_odom_target_msg);
        return result_enemy_target;
    }
    if (msg.red_num >= 2)
    {
        ROS_INFO("red_num = 2");

        // 判断之前有没有打击过敌人, 如果之前没有打击过敌人, 选择距离近的敌人, 否则选择之前打击过的
        for (int i = 0; i < msg.num; i++)
        {
            if (msg.object[i].team.data == "red" + to_string(i))
            {
                enemy_index.push_back(i);
                enemy_self_distance.push_back(msg.object[enemy_index1].pose.position.x);
                enemy_last_distance.push_back(pow(msg.object[enemy_index1].globalpose.position.x - last_enemy_target_msg.object[0].globalpose.position.x, 2) + pow(msg.object[enemy_index1].globalpose.position.y - last_enemy_target_msg.object[0].globalpose.position.y, 2));
            }
        }

        if (last_enemy_target_msg.num == 0)
        {
            vector<float>::iterator smallest = min_element(enemy_self_distance.begin(), enemy_self_distance.end());
            int idx = distance(enemy_self_distance.begin(), smallest);

            ROS_INFO("red_num = 2, num = 0");
            // 没有打击过敌人, 选择相对距离近的敌人

            // 选择第一个敌人
            enemy_odom_target_msg.num = 1;
            enemy_odom_target_msg.red_num = 1;
            enemy_odom_target_msg.object.push_back(msg.object[idx]);
            ROS_INFO("OK18");

            enemy_odom_target_msg.object[0].pose.orientation.w = 1;
            enemy_odom_target_msg.object[0].globalpose.orientation.w = 1;
            result_enemy_target = enemy_odom_target_msg;

            pub_enemy_target.publish(enemy_odom_target_msg);
            return result_enemy_target;
        }
        else
        {
            ROS_INFO("red_num = 2, num != 0");
            vector<float>::iterator smallest = min_element(enemy_last_distance.begin(), enemy_last_distance.end());
            int idx = distance(enemy_last_distance.begin(), smallest);
            // 之前打击过敌人, 选择离之前的选择近的敌人
            ROS_INFO("OK22");

            enemy_odom_target_msg.num = 1;
            enemy_odom_target_msg.red_num = 1;
            enemy_odom_target_msg.object.push_back(msg.object[idx]);
            enemy_odom_target_msg.object[0].pose.orientation.w = 1;
            enemy_odom_target_msg.object[0].globalpose.orientation.w = 1;
            result_enemy_target = enemy_odom_target_msg;
            pub_enemy_target.publish(enemy_odom_target_msg);
            return result_enemy_target;
        }
    }
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

// 打击函数
GambalInfo RoboControl::ctl_stack_enemy()
{
    int armor_max_lost_num = 5;
    // result -> mode, yaw, pitch, global_z
    // 返回值, 包含 云台控制模式, 3个角度信息
    // armor检测和realsense检测和armor丢帧状态合起来有八种状态
    ROS_INFO("armor_lost_counter: %d", armor_lost_counter);

    // 2. realsense和armor都没有看到的时候, 并且丢帧数量小于 400, 维持云台角度
    // 3. realsense看到, armor没看到, 并且丢帧数量大于400帧, realsense引导云台转动
    if (armor_lost_counter > armor_max_lost_num && armor_info_msg.mode == 1)
    {
        // realsense和armor都没有看到的时候, 并且丢帧数量大于 400, 开始摇头
        // if (enemy_information.red_num == 0 && armor_info_msg.mode == 1)
        // {
        //     ROS_INFO("mode = 1");
        //     sent_mcu_gimbal_result.mode = 1;
        //     sent_mcu_gimbal_result.yaw = 0;
        //     sent_mcu_gimbal_result.pitch = 0;
        //     sent_mcu_gimbal_result.global_z = 0;

        //     first_in_realsense_flag = true;
        // }
        ROS_INFO("OK23");

        if (enemy_information.red_num > 0 && first_in_realsense_flag == true)
        {

            ROS_INFO("realsense detected");
            first_in_realsense_flag = false;
            int enemy_realsense_angle = 0;
            if (robo_ukf_enemy_information.orientation.z != 999)
            {
                enemy_realsense_angle = -robo_ukf_enemy_information.orientation.z * 180.0 / PI;
            }
            first_in_gimbal_angle = game_msg.gimbalAngleYaw;
            target_gimbal_angle = enemy_realsense_angle;

            sent_mcu_gimbal_result.mode = 3;
            sent_mcu_gimbal_result.yaw = enemy_realsense_angle * 100;
            sent_mcu_gimbal_result.pitch = 0;
            sent_mcu_gimbal_result.global_z = 0;
        }
        ROS_INFO("armor_lost_counter > 400");
        current_gimbal_angle = game_msg.gimbalAngleYaw;
        ROS_INFO("first_in: %f, target: %f, current: %f", first_in_gimbal_angle, target_gimbal_angle, current_gimbal_angle);
        if (abs(abs(current_gimbal_angle - first_in_gimbal_angle) - abs(target_gimbal_angle)) < 5)
        {
            first_in_armor_flag = true;
            armor_lost_counter = 0;
        }
    }
    if (armor_lost_counter <= armor_max_lost_num || armor_info_msg.mode > 1)
    {
        // 4. realsense看到, armor没看到, 并且丢帧数量小于400帧, realsense不管
        if (armor_info_msg.mode == 1)
        {
            ROS_INFO("lose armor");
            armor_lost_counter++;
            if (first_in_armor_flag == true)
            {
                sent_mcu_gimbal_result.mode = 2;
                sent_mcu_gimbal_result.yaw = 0;
                sent_mcu_gimbal_result.pitch = 32760;
                sent_mcu_gimbal_result.global_z = 0;
            }
            if (detected_armor_flag == true)
            {
                sent_mcu_gimbal_result.mode = 2;
                sent_mcu_gimbal_result.yaw = 0;
                sent_mcu_gimbal_result.pitch = 20;
                sent_mcu_gimbal_result.global_z = 0;
            }
        }

        // 只要 armor 检测到, 云台就转,j, 没有商量的余地, armor 丢帧的话需要商量一下
        // 3. realsense和armor都看到, 以armor的转动为主
        if (armor_info_msg.mode > 1)
        {
            ROS_INFO("detected armor");
            sent_mcu_gimbal_result.mode = 2;
            sent_mcu_gimbal_result.yaw = armor_info_msg.yaw + robo_ukf_enemy_information.orientation.w;
            sent_mcu_gimbal_result.pitch = armor_info_msg.pitch;
            sent_mcu_gimbal_result.global_z = armor_info_msg.global_z * 100;

            armor_lost_counter = 0;
            detected_armor_flag = true;
            first_in_armor_flag = false;
        }
        target_gimbal_angle = 1000;
        first_in_realsense_flag = true;
    }
    return sent_mcu_gimbal_result;
}

// TODO: 底盘只存在两种控制模式, 1. 人工给定确定点的控制模式, 2. 由敌人位置确定的控制模式，敌人位置确定的控制模式优先级最高, 然后才是人工给定的点的控制模式
VelInfo RoboControl::ctl_go_to_point(int mode, float goal_x, float goal_y, float goal_yaw)
{
    /*************************************************************************
    *  ctl_go_to_point()
    *  功能说明：去某个点
    *  参数说明：mode: 1 -> 根据给定点去某个点, 2 -> 根据地方位置去某个点(不使用realsense信息), 3 -> 根据地方位置去某个点(使用realsense信息)
    *  函数返回：返回下次去的位置
    *************************************************************************/
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0;
    target_pose.position.y = 0;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    if (mode == 1)
    {
        target_pose.position.x = goal_x;
        target_pose.position.y = goal_y;
        target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, goal_yaw);

        sent_mcu_vel_result.mode = 1;
    }
    if (mode == 2)
    {
        if (robo_ukf_enemy_information.orientation.z != 999)
        {
            goal_yaw = -robo_ukf_enemy_information.orientation.z * 180.0 / PI;
        }
        target_pose = ctl_track_enemy(goal_x, goal_y, goal_yaw);
        sent_mcu_vel_result.mode = 1;
    }
    if (mode == 3)
    {
    }
    sendNavGoal(target_pose);

    sent_mcu_vel_result.v_x = cmd_vel_msg.v_x;
    sent_mcu_vel_result.v_y = cmd_vel_msg.v_y;
    sent_mcu_vel_result.v_yaw = cmd_vel_msg.v_yaw;

    return sent_mcu_vel_result;
}

geometry_msgs::Pose RoboControl::ctl_track_enemy(double enemy_x, double enemy_y, double yaw)
{
    /*************************************************************************
    *  ctl_track_enemy()
    *  功能说明：跟踪敌军
    *  参数说明：enemy_{x,y} 敌军全局位置, yaw: 自身姿态
    *  函数返回：返回下次去的位置
    *************************************************************************/
    float x_coefficient = 0.4, y_coefficient = 0.6;
    float self_coefficient = 0.4, enemy_coefficient = 0.6;

    float self_x = robo_ukf_pose.position.x;
    float self_y = robo_ukf_pose.position.y;

    vector<float> dis_list;
    ROS_INFO("OK24");

    for (int i = 0; i < point_list.rows; i++)
    {
        ROS_INFO("find_enemy_self_closest_point_start: %d", i);
        float d_enemy_y = enemy_y - point_list.at<double>(i, 0) * 1.0 / 100;
        float d_enemy_x = enemy_x - point_list.at<double>(i, 1) * 1.0 / 100;

        float d_self_y = self_y - point_list.at<double>(i, 0) * 1.0 / 100;
        float d_self_x = self_x - point_list.at<double>(i, 1) * 1.0 / 100;

        float distance_enemy = sqrt(x_coefficient * d_enemy_x * d_enemy_x + y_coefficient * d_enemy_y * d_enemy_y);
        float distance_self = sqrt(x_coefficient * d_self_x * d_self_x + y_coefficient * d_self_y * d_self_y);

        if (distance_enemy < 0.5 || distance_enemy > 2.0)
        {
            distance_enemy = 1000.0;
        }

        dis_list.push_back(self_coefficient * distance_self + enemy_coefficient * distance_enemy);
        ROS_INFO("find_enemy_self_closest_point_end: %d", i);
    }
    ROS_INFO("OK25");

    vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end());
    ROS_INFO("OK26");

    int n = distance(dis_list.begin(), smallest);
    ROS_INFO("OK27");

    geometry_msgs::Pose target_pose;
    target_pose.position.x = point_list.at<double>(n, 1) / 100.0;
    target_pose.position.y = point_list.at<double>(n, 0) / 100.0;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw);
    return target_pose;
}