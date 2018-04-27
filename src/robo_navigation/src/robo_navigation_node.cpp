#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h> 
#include <tf/transform_datatypes.h> 
#include <tf/transform_listener.h>
#include "robo_navigation/global_planner.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/LaserScan.h"
#include "robo_navigation/PID.h"
using namespace std;



class RoboNav{
public:
    ros::NodeHandle* pnh;
    ros::Publisher pub_local_goal_pose;
    Floyd floyd;
    Mat arrArcs, point_list;
    vector<int> path;
    geometry_msgs::Pose cur_pose;
    geometry_msgs::Pose pre_goal;
    double fix_angle;
    PIDctrl pid_x;
    PIDctrl pid_y;
    PIDctrl pid_yaw;
    tf::TransformListener* tf_;
    
    RoboNav();
    void init();
    void cb_tar_pose(const geometry_msgs::PoseConstPtr& msg);
    void cb_cur_pose(const nav_msgs::Odometry &msg);
    int findClosestPt(double x,double y);
    void get_vel(geometry_msgs::Twist& msg_vel);
    void setFixAngle(geometry_msgs::Quaternion& qua);
};

RoboNav::RoboNav(){
    pnh=new ros::NodeHandle("");
    pub_local_goal_pose=pnh->advertise<geometry_msgs::PoseStamped>("nav/local_goal",1);
    tf_ = new tf::TransformListener();
}

void RoboNav::init(){
    FileStorage fs("/home/ubuntu/robot/src/robo_navigation/launch/matrix.xml", FileStorage::READ);
    fs["Matrix"] >> arrArcs;
    fs["Point"] >> point_list;
    floyd.loadMatrix(arrArcs);
    floyd.initFloydGraph();
    
    double Kp_linear,Ki_linear,Kd_linear;
    double limit_linear_max = 1.0;
    double Kp_angular,Ki_angular,Kd_angular;
    double limit_angular = 1.5;
    ros::param::param<double>("Kp_linear",Kp_linear,1.5);
    ros::param::param<double>("Ki_linear",Ki_linear,0);
    ros::param::param<double>("Kd_linear",Kd_linear,0);
    ros::param::param<double>("Kp_angular",Kp_angular,2.0);
    ros::param::param<double>("Ki_angular",Ki_angular,0);
    ros::param::param<double>("Kd_angular",Kd_angular,0);
    
    pid_x.init(Kp_linear,Ki_linear,Kd_linear,limit_linear_max);
    pid_y.init(Kp_linear,Ki_linear,Kd_linear,limit_linear_max);
    pid_yaw.init(Kp_angular,Ki_angular,Kd_angular,limit_angular);
}
void RoboNav::cb_tar_pose(const geometry_msgs::PoseConstPtr& msg){
   // ROS_INFO("pre-tar %f  %f",pre_goal.position.x - msg->position.x ,pre_goal.position.y - msg->position.y);
    if (cur_pose.position.y==0 & cur_pose.position.x==0)
	return;
   
    if ((abs(pre_goal.position.x - msg->position.x )>0.1 ) || (abs(pre_goal.position.y - msg->position.y )>0.1)){
	    
    int start_pt=findClosestPt(cur_pose.position.y,cur_pose.position.x);
    int end_pt=findClosestPt(msg->position.y,msg->position.x);
    floyd.calcPath(start_pt,end_pt);
    ROS_INFO("start %f  %f  end   %f  %f ",cur_pose.position.x,cur_pose.position.y,msg->position.x,msg->position.y);
    ROS_INFO("GET GOAL start: %d  end: %d",start_pt,end_pt);
    floyd.printPath();
    path.assign(floyd.path.begin(), floyd.path.end()); 
    pre_goal.position.x =msg->position.x;
    pre_goal.position.y =msg->position.y;
    setFixAngle(cur_pose.orientation);
    }
}

void RoboNav::cb_cur_pose(const nav_msgs::Odometry& msg){
cur_pose = msg.pose.pose;
}

int RoboNav::findClosestPt(double x,double y){
    vector<float> dis_list;
    //cout<<point_list<<endl;
    for (int i=0;i<point_list.rows;i++){
        float dx=x-point_list.at<double>(i,0)*1.0/100;
        float dy=y-point_list.at<double>(i,1)*1.0/100;
        float distance=sqrt(dx*dx+dy*dy);
        dis_list.push_back(distance);
    }
    
    vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end()); 
    
    int n=distance(dis_list.begin(), smallest);
    return n;
}


void RoboNav::get_vel(geometry_msgs::Twist& msg_vel)
{
    
    double limit_linear_min=0.1;
    
    double vel_x = 0;
    double vel_y = 0;
    double vel_yaw = 0;
    if (path.size() > 0) {
        int cur_local_goal = path[0];
        double cur_local_goal_y =point_list.at<double>(cur_local_goal, 0) * 1.0 / 100;
        double cur_local_goal_x =point_list.at<double>(cur_local_goal, 1) * 1.0 / 100;
        double cur_yaw=tf::getYaw(cur_pose.orientation);
	
//         tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
//                                              tf::Vector3(cur_local_goal_x,cur_local_goal_y,0)),
//                                  ros::Time(), "odom");
//         tf::Stamped<tf::Pose> pose;
// 	
//         try
//         {
//         
//         tf_->transformPose("base_link", ident, pose);
//         }
//         catch(tf::TransformException& e)
//         {
//         ROS_ERROR("Couldn't transform "
//                     "even though the message notifier is in use");
//         return;
//         }
	
	
	
        double dx = (cur_local_goal_x- cur_pose.position.x)*cos(cur_yaw)+(cur_local_goal_y - cur_pose.position.y)*sin(cur_yaw);
        double dy =-(cur_local_goal_x- cur_pose.position.x)*sin(cur_yaw)+(cur_local_goal_y - cur_pose.position.y)*cos(cur_yaw);
        double dyaw=fix_angle-cur_yaw;
	geometry_msgs::PoseStamped pose_local;
	pose_local.header.stamp=ros::Time::now();
	pose_local.header.frame_id="base_link";
	pose_local.pose.position.x=dx;
	pose_local.pose.position.y=dy;
	pose_local.pose.position.y=0;
	pose_local.pose.orientation.x=0;
	pose_local.pose.orientation.y=0;
	pose_local.pose.orientation.z=0;
	pose_local.pose.orientation.w=1;
	pub_local_goal_pose.publish(pose_local);
	
	    ROS_INFO("dx %f dy %f  dyaw %f ",dx,dy,dyaw);
        if (dyaw>6.28) dyaw=dyaw-6.28;
        if (dyaw<-6.28) dyaw=dyaw+6.28;
	    //ROS_INFO("angle: %f  fix angle : %f   dyaw %f",cur_yaw,fix_angle,dyaw);
        ROS_INFO("num: %d  tar_x %f, tar_y %f,cur_x %f , cur_y %f, diff_x %f, diff_y %f",cur_local_goal, cur_local_goal_x, cur_local_goal_y,
          cur_pose.position.x, cur_pose.position.y, dx, dy);
        if (abs(dx) < 0.05 && abs(dy) < 0.05) 
	    {
            path.erase(path.begin());
	    
        }
        else 
	    {
            vel_x=pid_x.calc(dx);
            vel_y=pid_y.calc(dy);
        
            if (vel_x>0 && vel_x<limit_linear_min) vel_x=limit_linear_min;
            if (vel_x<0 && vel_x>-limit_linear_min) vel_x=-limit_linear_min;
            
                if (vel_y>0 && vel_y<limit_linear_min) vel_y=limit_linear_min;
            if (vel_y<0 && vel_y>-limit_linear_min) vel_y=-limit_linear_min;
            
            if (abs(dx) < 0.05) vel_x=0;
            if (abs(dy) < 0.05) vel_y=0;
            
                vel_yaw=pid_yaw.calc(dyaw);
            if (abs(dyaw)<0.1) vel_yaw=0;
        }
    }

    msg_vel.linear.x = vel_x;
    msg_vel.linear.y = vel_y;
    msg_vel.angular.z = vel_yaw;
}

void RoboNav::setFixAngle(geometry_msgs::Quaternion& qua){
    
    fix_angle=tf::getYaw(qua);
}

#define OFFSET 20    //rplidar front offset

/*obs_point record the valid obs point in four directions
 * for rplidar (360 degree and 360 points), point steps by one degree
 *  obs_point[0][]  -----forward  [0][0]=20
 *  obs_point[1][]  -----left
 *  obs_point[2][]  -----backward
 *  obs_point[3][]  -----right
 *  
 */
double obs_point[4][3]={0}; //90,+-60

double Filter_ScanData(int index, const sensor_msgs::LaserScan::ConstPtr& sscan)
{
    int Cindex=index;
    double data=0;
    int m=0;
        while(1)
        {
            if(index+m<0) Cindex=360+index+m;
            else if(index+m>=360) Cindex=index+m-360;
            else Cindex=index+m;
            if(sscan->intensities[Cindex]==47)
            {
                data=sscan->ranges[Cindex];
                break;
            }

            if(index-m<0) Cindex=360+index-m;
            else if(index-m>=360) Cindex=index-m-360;
            else Cindex=index-m;
            if(sscan->intensities[Cindex]==47)
            {
                data=sscan->ranges[Cindex];
                break;
            }
            m++;
        }

    if(data<0.0)  data=8.0;
    else if(data>8.0)   data=8.0;

    return data;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i=0;i<4;i++)
    {
        //right
        int index_r=OFFSET+i*90-30;
        obs_point[i][0]=Filter_ScanData(index_r,scan);
        //center
        int index=OFFSET+i*90;
        obs_point[i][1]=Filter_ScanData(index,scan);
        //left
        int index_l=OFFSET+i*90+30;
        obs_point[i][2]=Filter_ScanData(index_l,scan);
	//ROS_INFO("No %d   %f %f  %f ",i, obs_point[i][0],obs_point[i][1],obs_point[i][2]);
    }
    
    
}

int main(int argc,char **argv){
    ros::init(argc,argv,"robo_navigation");
    ros::NodeHandle nh;

    RoboNav robo_nav;
    robo_nav.init();
    ros::Subscriber cb_tar_pose = nh.subscribe("base/goal", 1, &RoboNav::cb_tar_pose, &robo_nav);
    ros::Subscriber cb_cur_pose = nh.subscribe("odom", 1, &RoboNav::cb_cur_pose, &robo_nav);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate rate(50);
    while (ros::ok()){
	if (robo_nav.path.size()>0){
	    geometry_msgs::Twist msg_vel;
	    //ROS_INFO("vel x: %f y:%f",msg_vel.linear.x,msg_vel.linear.y);
	    robo_nav.get_vel(msg_vel);
	    pub_vel.publish(msg_vel);
	}
        ros::spinOnce();
        rate.sleep();
    }
}
