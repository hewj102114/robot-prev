#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "robo_navigation/global_planner.hpp"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/LaserScan.h"
using namespace std;



class RoboNav{
public:
    ros::NodeHandle* pnh;
    Floyd floyd;
    Mat arrArcs, point_list;
    vector<int> path;
    geometry_msgs::Pose cur_pose;
    RoboNav();
    void init();
    void cb_tar_pose(const geometry_msgs::PoseConstPtr& msg);
    void cb_cur_pose(const nav_msgs::Odometry &msg);
    int findClosestPt(int x,int y);
    void get_vel(double& vel_x,double& vel_y);
};

RoboNav::RoboNav(){
    pnh=new ros::NodeHandle("");
    
}

void RoboNav::init(){
    FileStorage fs("/home/ubuntu/robot/src/robo_navigation/launch/matrix.xml", FileStorage::READ);
    fs["Matrix"] >> arrArcs;
    fs["Point"] >> point_list;
    floyd.loadMatrix(arrArcs);
    floyd.initFloydGraph();
}
void RoboNav::cb_tar_pose(const geometry_msgs::PoseConstPtr& msg){
    int start_pt=findClosestPt(cur_pose.position.y,cur_pose.position.x);
    int end_pt=findClosestPt(msg->position.x,msg->position.y);
    floyd.calcPath(start_pt,end_pt);
    ROS_INFO("%f  %f ",msg->position.x,msg->position.y);
    ROS_INFO("GET GOAL start: %d  end: %d",start_pt,end_pt);
    floyd.printPath();
    path.assign(floyd.path.begin(), floyd.path.end()); 
}

void RoboNav::cb_cur_pose(const nav_msgs::Odometry& msg){
cur_pose = msg.pose.pose;
}

int RoboNav::findClosestPt(int x,int y){
    vector<float> dis_list;
    for (int i=0;i<point_list.rows;i++){
        float dx=x-point_list.at<int>(i,0)*1.0/100;
        float dy=y-point_list.at<int>(i,1)*1.0/100;
        float distance=sqrt(dx*dx+dy*dy);
        dis_list.push_back(distance);
    }
    
    vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end()); 
    cout<<"aaaaaaaaaaaaa  "<<*smallest<<endl;
    int n=distance(dis_list.begin(), smallest);
    return n;
}


void RoboNav::get_vel(double& vel_x, double& vel_y)
{
    double Kp=1;vel_x=0;vel_y=0;
    if (path.size()>0){
	int cur_local_goal=path[0];
	double cur_local_goal_y=point_list.at<int>(cur_local_goal,0)*1.0/100;
	double cur_local_goal_x=point_list.at<int>(cur_local_goal,1)*1.0/100;
	// double path_yaw=tan((cur_local_goal_y-cur_pose.position.y)/(cur_local_goal_x-cur_pose.position.x));
	// double car_yaw=tan((cur_local_goal_y-cur_pose.position.y)/(cur_local_goal_x-cur_pose.position.x));
	double dx=cur_local_goal_x-cur_pose.position.x;
	double dy=cur_local_goal_y-cur_pose.position.y;
	ROS_INFO("num: %d  cur_x %f, cur_y %f, tar_x %f , tar_y %f",cur_local_goal,cur_local_goal_x,cur_local_goal_y,cur_pose.position.x,cur_pose.position.y);
	if (dx<0.15 && dy <0.25){
	    path.erase(path.begin());
	    return;
	}
	vel_x=dx*Kp;vel_y=dy*Kp;
	if (vel_x>0.8) vel_x=0.8;
	if (vel_x<0.1) vel_x=0.1;
	if (vel_y>0.8) vel_y=0.8;
	if (vel_y<0.1) vel_y=0.1;
    }
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

    if(data<0.25)  data=8.0;
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
    }
}

int main(int argc,char **argv){
    ros::init(argc,argv,"robo_navigation");
    ros::NodeHandle nh;

    RoboNav robo_nav;

    ros::Subscriber cb_tar_pose = nh.subscribe("base/goal", 1, &RoboNav::cb_tar_pose, &robo_nav);
    ros::Subscriber cb_cur_pose = nh.subscribe("map/uwb/data", 1, &RoboNav::cb_cur_pose, &robo_nav);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, scanCallback);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate rate(10);
    while (ros::ok()){
	ROS_INFO("%d",robo_nav.path.size());
	if (robo_nav.path.size()>0){
	    geometry_msgs::Twist msg_vel;
	    robo_nav.get_vel(msg_vel.linear.x,msg_vel.linear.y);
	    pub_vel.publish(msg_vel);
	}
        ros::spinOnce();
        rate.sleep();
    }
}