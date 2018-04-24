#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "robo_navigation/global_planner.hpp"
#include <opencv2/opencv.hpp>
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
};

RoboNav::RoboNav(){
    pnh=new ros::NodeHandle("");
    
}

void RoboNav::init(){
    FileStorage fs("../draw_map/matrix.xml", FileStorage::READ);
    fs["Matrix"] >> arrArcs;
    fs["Point"] >> point_list;
    floyd.loadMatrix(arrArcs);
    floyd.initFloydGraph();
}
void RoboNav::cb_tar_pose(const geometry_msgs::PoseConstPtr& msg){
    int start_pt=findClosestPt(cur_pose.position.x,cur_pose.position.x);
    int end_pt=findClosestPt(msg->position.x,msg->position.x);
    floyd.calcPath(start_pt,end_pt);
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
    int n=distance(dis_list.begin(), smallest);
    return n;
}
int main(int argc,char **argv){
    ros::init(argc,argv,"robo_navigation");
    ros::NodeHandle nh;

    RoboNav robo_nav;

    ros::Subscriber cb_tar_pose = nh.subscribe("base/goal", 1, &RoboNav::cb_tar_pose, &robo_nav);
    ros::Subscriber cb_cur_pose = nh.subscribe("odom", 1, &RoboNav::cb_cur_pose, &robo_nav);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 1);

    ros::Rate rate(10);
    while (ros::ok()){

        ros::spinOnce();
        rate.sleep();
    }
}