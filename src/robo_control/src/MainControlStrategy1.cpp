#include <iostream>
#include "robo_control/robo_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_control_strategy_1");
    ros::NodeHandle nh;

    RoboControl robo_ctl;
    ros::Subscriber sub_armor_info = nh.subscribe("base/armor_info", 1, &RoboControl::cb_armorInfo, &robo_ctl);
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, &RoboControl::cb_cmd_vel, &robo_ctl);
    ros::Subscriber sub_move_base = nh.subscribe("move_base/feedback", 1, &RoboControl::cb_move_base, &robo_ctl);
    // ros::Subscriber sub_enemy_pose = nh.subscribe("enemy/odom_pose", 1, &RoboControl::cb_enemy_pose, &robo_ctl);
    ros::Subscriber sub_enemy_pnp_pose = nh.subscribe("enemy/pnp_pose", 1, &RoboControl::cb_enemy_pnp_pose, &robo_ctl);
    ros::Subscriber sub_robo_pose = nh.subscribe("odom", 1, &RoboControl::cb_ukf_pose, &robo_ctl);
    ros::Subscriber sub_nav_cur_goal = nh.subscribe("move_base/current_goal", 1, &RoboControl::cb_cur_goal, &robo_ctl);
    ros::Subscriber sub_move_base_status = nh.subscribe("move_base/status", 1, &RoboControl::cb_move_base_status, &robo_ctl);
    ros::Subscriber sub_another_robo_pose = nh.subscribe("/DeepWhuRobot1/odom", 1, &RoboControl::cb_another_robo_odom, &robo_ctl);
    ros::Subscriber sub_finish_navigation = nh.subscribe("nav_state", 1, &RoboControl::cb_finish_navigation, &robo_ctl);
    ros::Subscriber sub_enemy_information = nh.subscribe("infrared_detection/enemy_position", 1, &RoboControl::cb_enemy_information, &robo_ctl);
    ros::Subscriber sub_ukf_enemy_information = nh.subscribe("ukf/enemy", 1, &RoboControl::cb_ukf_enemy_information, &robo_ctl);
    ros::Subscriber sub_front_dis = nh.subscribe("front_dis", 1, &RoboControl::cb_front_dis, &robo_ctl);

    int init_flag = 1;
    geometry_msgs::Pose nav_goal; // goal of navigation
    int armor_lost_count = 0;
    int key_point_no = 1;
    clock_t start, end;
    robo_ctl.main_control_init();
    robo_ctl.read_xml_file();

    /* TODO: 2018-05-01 测试完整逻辑
	1. 到中心点 (使用自己写的导航包)
	2. 看到敌人(realsense) -> 敌我判断 -> 到敌人附近 -> 攻击
	3. 没有看到敌人 -> 继续巡图
	*/
    /* 重要参数: 
	gimbal 1:serach 2:shoot
	chassis 1:velcity 2:angle pose 3:init
	*/
    geometry_msgs::Pose target_pose;
    int work_state = 0;             // switch case 的状态位

    ros::Rate loop_rate(150);       // ROS 帧率
    while (ros::ok())
    {
        robo_ctl.readMCUData();
        switch (work_state)
        {
        /*************************************************************************
		*
		*  0. 抢占中点
		*
		*************************************************************************/
        case 0:
        {
            ROS_INFO("Stage 0: Go to center!");
            work_state = 1;
            break;
        }
        /*************************************************************************
		*
		*  1. 站位, 最简单策略, 固定一个站位点, 没有看到敌人就回到这个点
		*
		*************************************************************************/
        case 1:
        {
            ROS_INFO("Stage 1: Go to certain point!!!!!!");
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_go_to_point(1, 2.6, 3.1, 0);
            if (robo_ctl.last_enemy_target.red_num == 1)
            {
                work_state = 2;
            }
            break;
        }

        /*************************************************************************
		*
		*  2. 检测到敌人, 跟着打
		*
		*************************************************************************/
        case 2:
        {
            ROS_INFO("Stage 2: Tracking enemy!!!!!!");            
            robo_ctl.sent_mcu_vel_msg = robo_ctl.ctl_go_to_point(2, robo_ctl.last_enemy_target_pose.position.x, robo_ctl.last_enemy_target_pose.position.y, 0);
            if (robo_ctl.last_enemy_target.red_num == 0)
            {
                work_state = 1;
            }
            break;
        }
        default:
            break;
        }
        ROS_INFO("OK5");
        robo_ctl.last_enemy_target = robo_ctl.sendEnemyTarget(robo_ctl.enemy_information, robo_ctl.last_enemy_target);  // 目标会一直发送, 通过 'Nothing' 来判断有没有敌人
        ROS_INFO("OK6");
        robo_ctl.last_enemy_target_pose.position.x = robo_ctl.last_enemy_target.object[0].globalpose.position.x;
        ROS_INFO("OK7");
        robo_ctl.last_enemy_target_pose.position.y = robo_ctl.last_enemy_target.object[0].globalpose.position.y;        
        ROS_INFO("OK8");
        robo_ctl.sent_mcu_gimbal_msg = robo_ctl.ctl_stack_enemy();      // 云台一直转动, 无论干什么都是一直转动
        ROS_INFO("OK9");
        robo_ctl.sendMCUMsg(1, robo_ctl.sent_mcu_gimbal_msg.mode,
                            robo_ctl.sent_mcu_vel_msg.v_x, robo_ctl.sent_mcu_vel_msg.v_y, robo_ctl.sent_mcu_vel_msg.v_yaw,
                            robo_ctl.sent_mcu_gimbal_msg.yaw, robo_ctl.sent_mcu_gimbal_msg.pitch, robo_ctl.sent_mcu_gimbal_msg.global_z);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
