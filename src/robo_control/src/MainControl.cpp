#include <iostream>
#include "robo_control/robo_control.hpp"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robo_control");
	ros::NodeHandle nh;

	RoboControl robo_ctl;
	ros::Subscriber sub_armor_info = nh.subscribe("base/armor_info", 1, &RoboControl::cb_armorInfo, &robo_ctl);
	ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, &RoboControl::cb_cmd_vel, &robo_ctl);
	ros::Subscriber sub_move_base = nh.subscribe("move_base/feedback", 1, &RoboControl::cb_move_base, &robo_ctl);
	ros::Subscriber sub_enemy_pose = nh.subscribe("enemy/odom_pose", 1, &RoboControl::cb_enemy_pose, &robo_ctl);
	ros::Subscriber sub_enemy_pnp_pose = nh.subscribe("enemy/pnp_pose", 1, &RoboControl::cb_enemy_pnp_pose, &robo_ctl);
	ros::Subscriber sub_robo_pose = nh.subscribe("odom", 1, &RoboControl::cb_ukf_pose, &robo_ctl);
	ros::Subscriber sub_nav_cur_goal = nh.subscribe("move_base/current_goal", 1, &RoboControl::cb_cur_goal, &robo_ctl);
	ros::Subscriber sub_move_base_status = nh.subscribe("move_base/status", 1, &RoboControl::cb_move_base_status, &robo_ctl);
	ros::Subscriber sub_another_robo_pose = nh.subscribe("/DeepWhuRobot1/odom", 1, &RoboControl::cb_another_robo_odom, &robo_ctl);
	ros::Subscriber sub_finish_navigation = nh.subscribe("nav_state", 1, &RoboControl::cb_finish_navigation, &robo_ctl);
	ros::Subscriber sub_enemy_information = nh.subscribe("infrared_detection/enemy_position", 1, &RoboControl::cb_enemy_information, &robo_ctl);
	ros::Subscriber sub_ukf_enemy_information = nh.subscribe("ukf/enemy", 1, &RoboControl::cb_ukf_enemy_information, &robo_ctl);


	int init_flag = 1;
	geometry_msgs::Pose nav_goal; // goal of navigation
	int armor_lost_count = 0;
	int key_point_no = 1;
	clock_t start, end;
	robo_ctl.read_xml_file();

	while (0)
	{
		robo_ctl.readMCUData();
		KeyPoint goalKeyPoint = robo_ctl.KEY_POINT[key_point_no];
		KeyPoint currentPosition = robo_ctl.KEY_POINT[key_point_no];

		float x_threshold = 0.5;
		float y_threshold = 0.5;
		float yaw_threshold = 0.2;

		// if robo doesn't arrive goal point, loop

		goalKeyPoint = robo_ctl.KEY_POINT[key_point_no];

		nav_goal.position.x = goalKeyPoint.x;
		nav_goal.position.y = goalKeyPoint.y;

		float angle = 0.0;
		if (goalKeyPoint.yaw != 0.0 || goalKeyPoint.yaw != 180 * PI / 180.0)
		{
			angle = 0.0;
		}
		nav_goal.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
		robo_ctl.sendNavGoal(nav_goal);

		currentPosition.x = robo_ctl.robo_ukf_pose.position.x;
		currentPosition.y = robo_ctl.robo_ukf_pose.position.y;
		//currentPosition.yaw = robo_ctl.robo_ukf_pose.orientation;

		float x_error = abs(currentPosition.x - goalKeyPoint.x);
		float y_error = abs(currentPosition.y - goalKeyPoint.y);
		float yaw_error = 0;
		//float yaw_error = abs(currentPosition.yaw - goalKeyPoint.yaw);
		ROS_INFO("Goal: %d %f %f %f, error: %f %f %f",
				 key_point_no, goalKeyPoint.x, goalKeyPoint.y, goalKeyPoint.yaw,
				 x_error, y_error, yaw_error);

		// if robo finish move, give it next key point
		if (robo_ctl.judgeKeyPointPosition(currentPosition, goalKeyPoint, x_threshold, y_threshold, yaw_threshold))
		{
			ROS_INFO("robo_ctl.nav_status");
			key_point_no = key_point_no + 2;
			start = clock();
		}
		end = clock();
		ROS_INFO("TIME: %f", (double)(end - start) / CLOCKS_PER_SEC);
		if ((double)(end - start) / CLOCKS_PER_SEC > 10)
		{
			key_point_no = key_point_no + 1;
			start = clock();
		}
		// point num = 30
		if (key_point_no >= 25)
		{
			key_point_no = 1;
		}

		robo_ctl.sendMCUMsg(1, 2, robo_ctl.cmd_vel_msg.v_x,
							robo_ctl.cmd_vel_msg.v_y,
							robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
		ros::spinOnce();
	}

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
	int work_state = 0;
	ros::Rate loop_rate(150);
	bool first_in = true;
	long long int count = 0;
	float first_in_gimbal_angle = 0;
	float current_gimbal_angle = 0;
	float target_gimbal_angle = 1000;
    int   last_armor_info_msg_mode = 0;
	ros::Time first_in_armor_timer = ros::Time::now();
	bool first_in_armor_flag = false;
	bool timeout_searching_flag = false;
	int lose_gimbal_count = 0;
	int lose_frame_count = 0;
	bool detected_armor_flag = false;
	while (ros::ok())
	{
		// 读取 MCU 数据
		robo_ctl.readMCUData();
		switch (work_state)
		{
			ROS_INFO("======================================================\n================================================");
		/*************************************************************************
		*
		*  0. 抢占中点
		*
		*************************************************************************/
		case 0:
		{
			ROS_INFO("Stage 0: Go to center!!!!!!");

			target_pose.position.x = 4.0;
			target_pose.position.y = 2.5;
			target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 3.14);

			robo_ctl.sendNavGoal(target_pose);

			// 到达中点并停留 n 秒, 结束本阶段

			if (robo_ctl.finish_navigation.data)
			{
				ROS_INFO("Arrived goal!!!!");
				start = clock();
				while (1)
				{
					end = clock();
					if ((double)(end - start) / CLOCKS_PER_SEC > 5)
					{
						robo_ctl.finish_goto_center = true;
						break;
					}
					robo_ctl.readMCUData();
					robo_ctl.sendMCUMsg(1, 1, robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
					ros::spinOnce();
				}
			}
			else
			{
				ROS_INFO("Going to goal!!!!");
			}

			if (robo_ctl.finish_goto_center == true)
			{
				// 没有发现敌人
				if (true)
				{
					work_state = 1;
				}
				else
				{
					work_state = 2;
				}
			}
			robo_ctl.sendMCUMsg(1, 1, robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
			break;
		}

		/*************************************************************************
		*
		*  1. 没有发现敌人
		*
		*************************************************************************/
		case 1:
		{
			/* 
			TODO: 刚到达中点的巡图: 按照固定顺序进行
			TODO: 丢失敌人之后的巡图: 根据敌人丢失的位置巡图
			go_on_patrol(flag, current_position, enemy_position)
			*/
			ROS_INFO("Stage 1: Not find enemy, finding enemy!!!!!!");
			robo_ctl.go_on_patrol(1, robo_ctl.key_point_count, 0, 0);
			ros::Time start_time = ros::Time::now();
			ros::Duration timeout(0.1); // Timeout of 2 seconds
			while (ros::Time::now() - start_time < timeout)
			{
				robo_ctl.readMCUData();
				robo_ctl.sendMCUMsg(1, 1, robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
				ros::spinOnce();
			}
			if (robo_ctl.finish_navigation.data == true)
			{
				ROS_INFO("Arrived goal!!!!");
				robo_ctl.key_point_count++;
				if (robo_ctl.key_point_count > 3)
				{
					robo_ctl.key_point_count = 0;
				}
			}
			else
			{
				ROS_INFO("Going to goal!!!!");
			}
			if (robo_ctl.enemy_information.num > 0)
			{
				work_state = 2;
			}
			robo_ctl.sendMCUMsg(1, 1, robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
			break;
		}

		/*************************************************************************
		*
		*  2. 发现敌人
		*
		*************************************************************************/
		case 2:
		{
			ROS_INFO("Stage 2: Find enemy, close to and stack enemy!!!!!!");

			int target_num = robo_ctl.find_enemy_self_closest_point(robo_ctl.enemy_odom_pose.position.x, 
																	robo_ctl.enemy_odom_pose.position.y, 
																	robo_ctl.robo_ukf_pose.position.x, 
																	robo_ctl.robo_ukf_pose.position.y);
			geometry_msgs::Pose target_pose;
			target_pose.position.x = robo_ctl.point_list.at<double>(target_num, 0);
			target_pose.position.y = robo_ctl.point_list.at<double>(target_num, 1);
			target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
			robo_ctl.sendNavGoal(target_pose);
			ros::Time start_time = ros::Time::now();
			ros::Duration timeout(0.1); // Timeout of 2 seconds
			while (ros::Time::now() - start_time < timeout)
			{
				robo_ctl.readMCUData();
				robo_ctl.sendMCUMsg(1, 1, robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
				ros::spinOnce();
			}
			if (robo_ctl.finish_navigation.data == true)
			{
				robo_ctl.sendMCUMsg(1, 1, 0, 0, 0, 0, 0, 0);
				work_state = 3;
			}
			else
			{
				robo_ctl.sendMCUMsg(1, 1, robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
			}
			break;
		}
		/*************************************************************************
		*
		*  3. Realsense 预瞄准
		*
		*************************************************************************/
		case 3:
		{
			ROS_INFO("Stage 3: Close to enemy, stacking enemy!!!!!!");
    		robo_ctl.enemy_odom_pose.orientation.w = 1;
			robo_ctl.sendEnemyTarget(robo_ctl.enemy_odom_pose);
			float enemy_self_angle = 0;
			if (robo_ctl.robo_ukf_enemy_information.orientation.z != 999)
			{
				enemy_self_angle = -robo_ctl.robo_ukf_enemy_information.orientation.z * 180.0 / PI;
			}
			
			ROS_INFO("predicted angle: %f", enemy_self_angle);
			count ++;
			if (robo_ctl.enemy_information.num > 0 && first_in == true && count % 1 == 0 && robo_ctl.robo_ukf_enemy_information.orientation.z != 999)
			{
				ROS_INFO("sent angle information!!!!!!!!!!");
				first_in = false;
				first_in_gimbal_angle = robo_ctl.game_msg.gimbalAngleYaw;
				target_gimbal_angle = enemy_self_angle;
				// ROS_INFO("=============================================================\n========================================================================\n====================================================================\n=======================================================================\n==========================================================================\n");
				robo_ctl.sendMCUMsg(1, 
									3, 
									0, 
									0, 
									0, 
									enemy_self_angle * 100, 
									0, 
									0);
			}
			// 云台转动完成, 跳转到装甲板识别
			current_gimbal_angle = robo_ctl.game_msg.gimbalAngleYaw;
			ROS_INFO("first_in_gimbal_angle: %f, target_gimbal_angle: %f, current_gimbal_angle: %f", first_in_gimbal_angle, target_gimbal_angle, current_gimbal_angle);
			if (abs(abs(current_gimbal_angle - first_in_gimbal_angle) - abs(target_gimbal_angle)) < 5)
			{
				work_state = 4;
				first_in_armor_flag = true;
				detected_armor_flag = false;
				first_in_armor_timer = ros::Time::now();
			}
			break;
		}
		/*************************************************************************
		*
		*  4. 装甲板识别
		*
		*************************************************************************/
		case 4:
		{
			ROS_INFO("Stage 4: Detect armor, stacking enemy!!!!!!");
			// while(first_in_armor_flag && robo_ctl.armor_info_msg.mode == 1)
			// {
			// 	ROS_INFO("Stage 4: searching");
			// 	robo_ctl.readMCUData();
			// 	robo_ctl.sendMCUMsg(1,
			// 						1,
			// 						0,
			// 						0,
			// 						0,
			// 						robo_ctl.armor_info_msg.yaw,
			// 						robo_ctl.armor_info_msg.pitch,
			// 						robo_ctl.armor_info_msg.global_z * 100);
			// 	ros::Duration timeout(10);
			// 	ros::Time current_armor_timer = ros::Time::now();
			// 	if (current_armor_timer - first_in_armor_timer > timeout)
			// 	{
			// 		first_in_armor_flag = false;
			// 		timeout_searching_flag = true;
			// 		break;
			// 	}
			// 	if (robo_ctl.armor_info_msg.mode == 2 || robo_ctl.armor_info_msg.mode == 3)
			// 	{
			// 		first_in_armor_flag = false;
			// 		break;
			// 	}
			// 	ros::spinOnce();
			// }
			if(robo_ctl.armor_info_msg.mode == 2 || robo_ctl.armor_info_msg.mode == 3)
			{
				float ukf_predicted_yaw = robo_ctl.robo_ukf_enemy_information.orientation.w;
				ROS_INFO("Detected armor!!!!!");
				lose_gimbal_count = 0;
				detected_armor_flag = true;
				first_in_armor_flag = false;
				// first_in_armor_flag = false;
				robo_ctl.sendMCUMsg(1,
									2,
									0,
									0,
									0,
									robo_ctl.armor_info_msg.yaw + ukf_predicted_yaw * 100,
									robo_ctl.armor_info_msg.pitch,
									robo_ctl.armor_info_msg.global_z * 100);
			}
			// if (robo_ctl.armor_info_msg.mode == 1 || timeout_searching_flag)
			if (robo_ctl.armor_info_msg.mode == 1)			
			{
				ROS_INFO("No detected armor!!!!!");
				lose_frame_count++;
				lose_gimbal_count++;
				// realsense 切换过来
				if (first_in_armor_flag == true)
				{
					robo_ctl.sendMCUMsg(1,
										2,
										0,
										0,
										0,
										0,
										32760,
										0);
				}
				// 装甲板检测丢帧
				if (detected_armor_flag == true)
				{
					robo_ctl.sendMCUMsg(1,
										2,
										0,
										0,
										0,
										0,
										5,
										0);
				}
				// first_in_armor_flag = false;

			}
			// if(timeout_searching_flag == true || (robo_ctl.armor_info_msg.mode == 1 && (last_armor_info_msg_mode == 2 || last_armor_info_msg_mode == 3)))
			// if((robo_ctl.armor_info_msg.mode == 1 && (last_armor_info_msg_mode == 2 || last_armor_info_msg_mode == 3)))
			if (lose_gimbal_count > 400)
			{
				lose_gimbal_count = 0;
				// first_in_armor_flag = false;
				work_state = 4;
				target_gimbal_angle = 1000;
				first_in = true;
			}
			last_armor_info_msg_mode = robo_ctl.armor_info_msg.mode;


			// robo_ctl.sendMCUMsg(1,
			// 					2,
			// 					0,
			// 					0,
			// 					0,
			// 					robo_ctl.armor_info_msg.yaw,
			// 					robo_ctl.armor_info_msg.pitch,
			// 					robo_ctl.armor_info_msg.global_z * 100);
			// if (robo_ctl.enemy_information.num == 0 && robo_ctl.armor_info_msg.mode == 1)
			// {
			// 	work_state = 3;
			// 	target_gimbal_angle = 1000;
			// }
			break;
		}
		case 5:
			robo_ctl.readMCUData();
			ROS_INFO("Stage 5: Testing!!!!!!");
    		robo_ctl.enemy_odom_pose.orientation.w = 1;
			robo_ctl.sendEnemyTarget(robo_ctl.enemy_odom_pose);
			robo_ctl.sendMCUMsg(1, 2, 0, 0, 0, 0, 0, 0);
			break;

		default:
			break;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	//gimbal 1:serach 2:shoot
	//chassis 1:velcity 2:angle pose 3:init
	int keypoint_num = 1;
	while (ros::ok())
	{
		robo_ctl.readMCUData();
		// go to center of playground
		if (robo_ctl.init_flag < 4)
		{
			ROS_INFO("INIT");
			keypoint_num = 5;
			robo_ctl.sendMCUMsg(3, 1, 0, 0, 0, 0, 0, 0);
			ros::Duration(0.1).sleep();
		}
		else
		{
			//find enemy by realsense
			if (robo_ctl.enemy_found_pnp_flag == 1)
			{
				ROS_INFO("MODE :  Find Enemy pnp   %d", robo_ctl.enemy_found_pnp_flag);
				int point_num = robo_ctl.getClosestKeyPoint(robo_ctl.enemy_odom_pnp_pose.position.x, robo_ctl.enemy_odom_pnp_pose.position.y);
				geometry_msgs::Pose target_pose;
				target_pose.position.x = robo_ctl.KEY_POINT[point_num].x;
				target_pose.position.y = robo_ctl.KEY_POINT[point_num].y;
				target_pose.orientation = robo_ctl.robo_ukf_pose.orientation;
				robo_ctl.sendNavGoal(target_pose);
				robo_ctl.enemy_found_pnp_flag = 0;
			}

			if (robo_ctl.armor_ready_flag == 1)
			{
				if (robo_ctl.armor_info_msg.mode > 0)
				{
					if (robo_ctl.armor_info_msg.global_z > 0.5)
					{
						ROS_INFO("MODE :  Find Enemy Far  %f  %f  %f",
								 robo_ctl.cmd_vel_msg.v_x,
								 robo_ctl.cmd_vel_msg.v_y,
								 robo_ctl.cmd_vel_msg.v_yaw);
						robo_ctl.sendMCUMsg(1, 
											2, 
											robo_ctl.cmd_vel_msg.v_x, 
											robo_ctl.cmd_vel_msg.v_y, 
											robo_ctl.cmd_vel_msg.v_yaw, 
											robo_ctl.armor_info_msg.yaw, 
											robo_ctl.armor_info_msg.pitch, 
											robo_ctl.armor_info_msg.global_z * 100);
					}
					else
					{
						ROS_INFO("MODE :  Find Enemy Close  %f  %f  %f   %f",
								 robo_ctl.cmd_vel_msg.v_x, 
								 robo_ctl.cmd_vel_msg.v_y,
								 robo_ctl.cmd_vel_msg.v_yaw, 
								 robo_ctl.armor_info_msg.global_z);
						robo_ctl.sendMCUMsg(1, 
											2, 
											0, 
											0, 
											0, 
											robo_ctl.armor_info_msg.yaw, 
											robo_ctl.armor_info_msg.pitch, 
											robo_ctl.armor_info_msg.global_z * 100);
					}
					armor_lost_count = 0;
				}
				else
				{
					armor_lost_count++;
					if (armor_lost_count > 150)
					{
						if (armor_lost_count > 400)
						{
							if (robo_ctl.keyPointNav(keypoint_num))
							{
								ROS_INFO("Reach No %d Point", keypoint_num);
								keypoint_num += 1;
								if (keypoint_num >= 22)
								{
									keypoint_num = 1;
								}
							}
							else
							{
								ROS_INFO("Gong to  No %d Point", keypoint_num);
							}
						}

						ROS_INFO("MODE :  Search  %f  %f  %f", robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw);

						robo_ctl.sendMCUMsg(1, 1, robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
						// msg_tomcu.setMsg(2,0*100,0*100,0.2*kyaw,0,0,0);
					}
					else
					{

						ROS_INFO("MODE :  Enemy Lost  %f  %f  %f",
								 robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw);
						robo_ctl.sendMCUMsg(1, 2, robo_ctl.cmd_vel_msg.v_x, robo_ctl.cmd_vel_msg.v_y, robo_ctl.cmd_vel_msg.v_yaw, 0, 0, 0);
					}
				}
				// cout<<"vx: "<<chassis_cmd_vel.v_x<<"
				// vy:"<<chassis_cmd_vel.v_y<<" v_th:"<<chassis_cmd_vel.v_yaw<<endl;
			}
		}
		ros::spinOnce();
	}
}
