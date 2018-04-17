#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>

#include <std_msgs/Int16.h>
#include "control/serial.h"
#include "whurobot_msgs/GameInfo.h"
#include "whurobot_msgs/ArmorInfo.h"

#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;
#define PI 3.1415926535898

//msg
int serial_ready_flag;
struct ArmorInfo
{
    int mode;
    float image_dx;
    float image_dy;
    float global_z;
    float pitch;
    float yaw;
}armor_info;

struct ChassisCMD{
    float v_x;
    float v_y;
    float v_yaw;
    
}chassis_cmd_vel;

void cb_armorInfo(const whurobot_msgs::ArmorInfo& msg)
{
    armor_info.mode=msg.mode;
    armor_info.image_dx=msg.pose_image.x;
    armor_info.image_dy=msg.pose_image.y;
    armor_info.global_z=msg.pose_global.position.z;
    armor_info.pitch=msg.angle.y;
    armor_info.yaw=msg.angle.x;
    serial_ready_flag=1;
}


void cb_cmd_vel(const geometry_msgs::Twist& msg)
{
    chassis_cmd_vel.v_x=msg.linear.x;
    chassis_cmd_vel.v_y=msg.linear.y;
    chassis_cmd_vel.v_yaw=msg.angular.z;
    
}

int nav_mode=-1;


void cb_move_base_fb(const move_base_msgs::MoveBaseActionFeedback& msg)
{
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
    
    nav_mode=msg.status.status;
}

float enemy_pose_x,enemy_pose_y;
int enemy_flag=0;
void cb_enemy_pose(const geometry_msgs::TransformStamped& msg){
    enemy_pose_x=msg.transform.translation.x;
    enemy_pose_y=msg.transform.translation.y;
    enemy_flag=1;
}

geometry_msgs::Pose robo_pose;
void cb_robo_pose(const nav_msgs::Odometry& msg){
    robo_pose=msg.pose.pose;
    
}

int main(int argc,char** argv)
{
    
    ros::init(argc,argv,"control");
    ros::NodeHandle nh;
    
    ros::Subscriber sub_armor_info=nh.subscribe("armor_info",1,&cb_armorInfo);
    ros::Subscriber sub_cmd_vel=nh.subscribe("cmd_vel",1,&cb_cmd_vel);
    ros::Subscriber sub_move_base=nh.subscribe("move_base/feedback",1,&cb_move_base_fb);
    ros::Subscriber sub_enemy_pose=nh.subscribe("enemy/odom_pose",1,&cb_enemy_pose);
    ros::Subscriber sub_robo_pose=nh.subscribe("odom",1,&cb_robo_pose);
    
    
    ros::Publisher pub_game_info=nh.advertise<whurobot_msgs::GameInfo>("game_info",1);
    whurobot_msgs::GameInfo game_msg;
    
    ros::Publisher pub_uwb_odom=nh.advertise<nav_msgs::Odometry>("map/uwb/data",1);
    nav_msgs::Odometry uwb_odom_msg;
    
    ros::Publisher pub_wheel_vel=nh.advertise<geometry_msgs::Vector3Stamped>("robo/wheel/data",1);
    geometry_msgs::Vector3Stamped wheel_vel_msg;
    
    ros::Publisher pub_imu_data=nh.advertise<sensor_msgs::Imu>("gimbal/imu/data",1);
    sensor_msgs::Imu imu_msg;
    
    ros::Publisher pub_pose_plan=nh.advertise<geometry_msgs::Pose>("base/goal_plan",1);
    geometry_msgs::Pose goal_plan_msg;
    
    tf2_ros::TransformBroadcaster gimbal_tf;
    geometry_msgs::TransformStamped gimbal_trans;
    
    tf2_ros::TransformBroadcaster camera_tf;
    geometry_msgs::TransformStamped camera_trans;
    
    tf2_ros::TransformBroadcaster odom_tf;
    geometry_msgs::TransformStamped odom_trans;
    
    Serial serial("/dev/ttyUSB0");
    serial.configurePort();
    struct RobotMsgToMCU msg_tomcu;
    struct RobotMsgFromMCU msg_frommcu;
    serial_ready_flag=0;
    //ros::Rate loop_rate(1000);
    
    char pre_uwb_ready_flag=0;
    float uwb_angle_bias=0;
    
    float current_goal_x,current_goal_y;
    //pose_point
     int armor_count=0;
    while (ros::ok())
    {
        //msg_frommcu
        if (serial.ReadData(msg_frommcu))
        {
            ros::Time time=ros::Time::now();
            game_msg.header.stamp=time;
            game_msg.header.frame_id="base_link";
            game_msg.remainingHP=msg_frommcu.remaining_HP;
            game_msg.attackArmorID=msg_frommcu.attack_armorID;
            game_msg.bulletCount=msg_frommcu.remaining_bullet;
            game_msg.bulletCount=msg_frommcu.uwb_yaw;
            game_msg.gimbalAngleYaw=msg_frommcu.gimbal_chassis_angle*1.0/100;
            game_msg.gimbalAnglePitch=(msg_frommcu.uwb_yaw*1.0/100);//0msg_frommcu.gimbal_pitch_angle*1.0/100
            pub_game_info.publish(game_msg);
            
            uwb_odom_msg.header.stamp=time;
            uwb_odom_msg.header.frame_id="odom";
            uwb_odom_msg.child_frame_id="base_link";
            uwb_odom_msg.pose.pose.position.x=msg_frommcu.uwb_x*1.0/100+0.13;
            uwb_odom_msg.pose.pose.position.y=msg_frommcu.uwb_y*1.0/100-0.09;
            uwb_odom_msg.pose.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,(msg_frommcu.uwb_yaw*1.0/100-uwb_angle_bias)*PI*2/360);
            if (pre_uwb_ready_flag != msg_frommcu.uwb_ready_flag){
                pub_uwb_odom.publish(uwb_odom_msg);
            }
            pre_uwb_ready_flag=  msg_frommcu.uwb_ready_flag;
            
            
            wheel_vel_msg.header.stamp=time;
            wheel_vel_msg.header.frame_id="base_link";
            wheel_vel_msg.vector.x=msg_frommcu.wheel_odom_x*1.0/10000;
            wheel_vel_msg.vector.y=msg_frommcu.wheel_odom_y*1.0/10000;
            pub_wheel_vel.publish(wheel_vel_msg);
            
            //       imu_msg.header.stamp=time;
            //       imu_msg.header.frame_id="imu_link";
            //       imu_msg.angular_velocity.x=msg_frommcu.imu_velocity_x*1.0/10000;
            //       imu_msg.angular_velocity.y=msg_frommcu.imu_velocity_y*1.0/10000;
            //       imu_msg.angular_velocity.z=msg_frommcu.imu_velocity_z*1.0/10000;
            //       imu_msg.linear_acceleration.x=msg_frommcu.imu_acceleration_x;
            //       imu_msg.linear_acceleration.y=msg_frommcu.imu_acceleration_y;
            //       imu_msg.linear_acceleration.z=msg_frommcu.imu_acceleration_z;
            //       pub_imu_data.publish(imu_msg);
            
            //       odom_trans.header.stamp = time;
            //       odom_trans.header.frame_id = "odom";
            //       odom_trans.child_frame_id = "base_link";
            //       odom_trans.transform.translation.x = msg_frommcu.uwb_x*1.0/100+0.13;
            //       odom_trans.transform.translation.y = msg_frommcu.uwb_y*1.0/100-0.09;
            //       odom_trans.transform.translation.z = 0.0;
            //       odom_trans.transform.rotation=tf::createQuaternionMsgFromRollPitchYaw(0,0,(msg_frommcu.uwb_yaw*1.0/100-uwb_angle_bias)*PI*2/360);
            //       odom_tf.sendTransform(odom_trans);
            
            
            gimbal_trans.header.stamp = time;
            gimbal_trans.header.frame_id = "base_link";
            gimbal_trans.child_frame_id = "gimbal_link";
            gimbal_trans.transform.translation.x =0.15;
            gimbal_trans.transform.translation.y =0;
            gimbal_trans.transform.translation.z = 0.0;
            gimbal_trans.transform.rotation=tf::createQuaternionMsgFromRollPitchYaw(0,msg_frommcu.gimbal_pitch_angle*1.0/100*PI*2/360,msg_frommcu.gimbal_chassis_angle*1.0/100*PI*2/360);
            gimbal_tf.sendTransform(gimbal_trans);
            
            camera_trans.header.stamp = time;
            camera_trans.header.frame_id = "gimbal_link";
            camera_trans.child_frame_id = "camera_link";
            camera_trans.transform.translation.x =0.15;
            camera_trans.transform.translation.y =0;
            camera_trans.transform.translation.z = 0.0;
            camera_trans.transform.rotation=tf::createQuaternionMsgFromRollPitchYaw(-1.57, 0,-1.57);
            camera_tf.sendTransform(camera_trans);
            
            msg_tomcu.clear();  
        }
        int kx=2.7,ky=2.7,kyaw=70;
        
        
        if (nav_mode==-1){
            //ROS_INFO("INIT ");
            goal_plan_msg.position.x=4;
            goal_plan_msg.position.y=2.5;
            goal_plan_msg.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
            pub_pose_plan.publish(goal_plan_msg);
            current_goal_x=4;
            current_goal_y=2.4;
        } 
        
        if(enemy_flag==1){
            if (abs(current_goal_x-enemy_pose_x)>0.15|| abs(current_goal_y-enemy_pose_y)>0.15  ){
                
                ROS_INFO("******************* Find Enemy  %f  %f",enemy_pose_x,enemy_pose_y);
                goal_plan_msg.position.x=enemy_pose_x-(enemy_pose_x-robo_pose.position.x)/4;
                goal_plan_msg.position.y=enemy_pose_y-(enemy_pose_y-robo_pose.position.y)/4;
                goal_plan_msg.orientation=robo_pose.orientation;
                pub_pose_plan.publish(goal_plan_msg);
                current_goal_x=enemy_pose_x;
                current_goal_y=enemy_pose_y;
                enemy_flag=0;
            }
        }
        
        
        if (serial_ready_flag==1){
            if (armor_info.mode>0){
                if (armor_info.global_z > 50){
                    ROS_INFO("MODE :  Find Enemy Far  %f  %f  %f",chassis_cmd_vel.v_x,chassis_cmd_vel.v_y,chassis_cmd_vel.v_yaw);
                    msg_tomcu.setMsg(2,chassis_cmd_vel.v_x*100,chassis_cmd_vel.v_y*100,chassis_cmd_vel.v_yaw*kyaw,armor_info.yaw,armor_info.pitch,armor_info.global_z);
                }
                else{
                    ROS_INFO("MODE :  Find Enemy Close  %f  %f  %f",chassis_cmd_vel.v_x,chassis_cmd_vel.v_y,chassis_cmd_vel.v_yaw);
                    msg_tomcu.setMsg(2,0,0,0,armor_info.yaw,armor_info.pitch,armor_info.global_z);
                }
                armor_count=0;
            }
            else {
                armor_count++;
                if (armor_count>150){
                    ROS_INFO("MODE :  Xunluo  %f  %f  %f",chassis_cmd_vel.v_x,chassis_cmd_vel.v_y,chassis_cmd_vel.v_yaw);
                msg_tomcu.setMsg(1,chassis_cmd_vel.v_x*100,chassis_cmd_vel.v_y*100,chassis_cmd_vel.v_yaw*kyaw,0,0,0);
                //msg_tomcu.setMsg(2,0*100,0*100,0.2*kyaw,0,0,0);
                }
                else{
                    ROS_INFO("MODE :  Enemy Lost  %f  %f  %f",chassis_cmd_vel.v_x,chassis_cmd_vel.v_y,chassis_cmd_vel.v_yaw);
                    msg_tomcu.setMsg(2,chassis_cmd_vel.v_x*100,chassis_cmd_vel.v_y*100,chassis_cmd_vel.v_yaw*kyaw,0,0,0);
                }
            }
            
            //cout<<"vx: "<<chassis_cmd_vel.v_x<<"   vy:"<<chassis_cmd_vel.v_y<<"   v_th:"<<chassis_cmd_vel.v_yaw<<endl;
            //cout<<"send"<<ros::Time::now()<<endl;
            serial.SendData(msg_tomcu);
            serial_ready_flag=0;
        }
        
        
        ros::spinOnce();
        //loop_rate.sleep();
    }
    //serial.Portclose();
}


