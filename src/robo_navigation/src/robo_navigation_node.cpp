#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
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
#define OFFSET 0 //rplidar front offset
#define DEFFENCE 0.40
#define DEFF_CORNER 0.55

class RoboNav
{
  public:
    ros::NodeHandle *pnh;
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
    tf::TransformListener *tf_;
    std_msgs::Bool state;
    /*obs_point record the valid obs point in four directions
     * for rplidar (360 degree and 360 points), point steps by one degree
     *  obs_point[0][]  -----forward  [0][0]=20
     *  obs_point[1][]  -----left
     *  obs_point[2][]  -----backward
     *  obs_point[3][]  -----right
     *  
     */
    double obs_min[4][2]; //90,+-60

    RoboNav();
    void init();
    void cb_tar_pose(const geometry_msgs::PoseConstPtr &msg);
    void cb_cur_pose(const nav_msgs::Odometry &msg);
    int findClosestPt(double x, double y);
    void get_vel(geometry_msgs::Twist &msg_vel);
    void setFixAngle(const geometry_msgs::Quaternion &qua);
    void cb_scan(const sensor_msgs::LaserScan::ConstPtr &scan);
    geometry_msgs::Pose adjustlocalgoal();
};

RoboNav::RoboNav()
{
    obs_min[4][2] = {0};
    pnh = new ros::NodeHandle("");
    pub_local_goal_pose = pnh->advertise<geometry_msgs::PoseStamped>("nav/local_goal", 1);
    tf_ = new tf::TransformListener();
}

void RoboNav::init()
{
    FileStorage fs("/home/ubuntu/robot/src/robo_navigation/launch/matrix.xml", FileStorage::READ);
    fs["Matrix"] >> arrArcs;
    fs["Point"] >> point_list;
    floyd.loadMatrix(arrArcs);
    floyd.initFloydGraph();

    state.data = false;

    double Kp_linear, Ki_linear, Kd_linear;
    double limit_linear_max = 1.0;
    double Kp_angular, Ki_angular, Kd_angular;
    double limit_angular = 1.5;
    ros::param::param<double>("Kp_linear", Kp_linear, 1.5);
    ros::param::param<double>("Ki_linear", Ki_linear, 0);
    ros::param::param<double>("Kd_linear", Kd_linear, 0);
    ros::param::param<double>("Kp_angular", Kp_angular, 2.0);
    ros::param::param<double>("Ki_angular", Ki_angular, 0);
    ros::param::param<double>("Kd_angular", Kd_angular, 0);

    pid_x.init(Kp_linear, Ki_linear, Kd_linear, limit_linear_max);
    pid_y.init(Kp_linear, Ki_linear, Kd_linear, limit_linear_max);
    pid_yaw.init(Kp_angular, Ki_angular, Kd_angular, limit_angular);


        cur_pose.position.x = 0;
        cur_pose.position.y = 0;
        cur_pose.position.y = 0;
        cur_pose.orientation.x = 0;
        cur_pose.orientation.y = 0;
        cur_pose.orientation.z = 0;
        cur_pose.orientation.w = 1;
}
void RoboNav::cb_tar_pose(const geometry_msgs::PoseConstPtr &msg)
{
    // ROS_INFO("pre-tar %f  %f",pre_goal.position.x - msg->position.x ,pre_goal.position.y - msg->position.y);
    if (cur_pose.position.y == 0 & cur_pose.position.x == 0)
        return;
    double dis = sqrt(pow(cur_pose.position.x - msg->position.x, 2) + pow(cur_pose.position.y - msg->position.y, 2));
    double dyaw = abs(tf::getYaw(cur_pose.orientation) - tf::getYaw(msg->orientation));
    if (dis < 0.5 && dyaw < 0.1)
        state.data = true;
    else
        state.data = false;
    //ROS_INFO("dis: %f", dis);
    if ((abs(pre_goal.position.x - msg->position.x) > 0.1) || (abs(pre_goal.position.y - msg->position.y) > 0.1))
    {
        int start_pt = findClosestPt(cur_pose.position.y, cur_pose.position.x);
        int end_pt = findClosestPt(msg->position.y, msg->position.x);
        if (start_pt != end_pt)
        {
            floyd.calcPath(start_pt, end_pt);
            ROS_INFO("start %f  %f  end   %f  %f ", cur_pose.position.x, cur_pose.position.y, msg->position.x, msg->position.y);
            ROS_INFO("GET GOAL start: %d  end: %d", start_pt, end_pt);
            floyd.printPath();
            path.assign(floyd.path.begin(), floyd.path.end());
            pre_goal.position.x = msg->position.x;
            pre_goal.position.y = msg->position.y;
        }
        else
            path.clear();
    }
    //isolated rotation control
    //setFixAngle(cur_pose.orientation);
    setFixAngle(msg->orientation); //orientation fixed all the time in every frame.
}

void RoboNav::cb_cur_pose(const nav_msgs::Odometry &msg)
{
    cur_pose = msg.pose.pose;
}

int RoboNav::findClosestPt(double x, double y)
{
    vector<float> dis_list;
    //cout<<point_list<<endl;
    for (int i = 0; i < point_list.rows; i++)
    {
        float dx = x - point_list.at<double>(i, 0) * 1.0 / 100;
        float dy = y - point_list.at<double>(i, 1) * 1.0 / 100;
        float distance = sqrt(dx * dx + dy * dy);
        dis_list.push_back(distance);
    }

    vector<float>::iterator smallest = min_element(dis_list.begin(), dis_list.end());

    int n = distance(dis_list.begin(), smallest);
    return n;
}

void RoboNav::get_vel(geometry_msgs::Twist &msg_vel)
{

    double vel_x = 0;
    double vel_y = 0;
    double vel_yaw = 0;
    
    if (path.size() > 0)
    {
        geometry_msgs::Pose cur_local_goal = adjustlocalgoal();
        double cur_local_goal_y = cur_local_goal.position.y;
        double cur_local_goal_x = cur_local_goal.position.x;
        double cur_yaw =tf::getYaw(cur_pose.orientation);
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

        double dx = (cur_local_goal_x - cur_pose.position.x) * cos(cur_yaw) + (cur_local_goal_y - cur_pose.position.y) * sin(cur_yaw);
        double dy = -(cur_local_goal_x - cur_pose.position.x) * sin(cur_yaw) + (cur_local_goal_y - cur_pose.position.y) * cos(cur_yaw);

        geometry_msgs::PoseStamped pose_local;
        pose_local.header.stamp = ros::Time::now();
        pose_local.header.frame_id = "base_link";
        pose_local.pose.position.x = dx;
        pose_local.pose.position.y = dy;
        pose_local.pose.position.y = 0;
        pose_local.pose.orientation.x = 0;
        pose_local.pose.orientation.y = 0;
        pose_local.pose.orientation.z = 0;
        pose_local.pose.orientation.w = 1;
        pub_local_goal_pose.publish(pose_local);

        //ROS_INFO("angle: %f  fix angle : %f   dyaw %f",cur_yaw,fix_angle,dyaw);
        //ROS_INFO(" tar_x %f, tar_y %f,cur_x %f , cur_y %f, diff_x %f, diff_y %f", cur_local_goal_x, cur_local_goal_y,
        //  cur_pose.position.x, cur_pose.position.y, dx, dy);
        if (abs(dx) < 0.05 && abs(dy) < 0.05)
        {
            path.erase(path.begin());
        }
        else
        {
            vel_x = pid_x.calc(dx);
            vel_y = pid_y.calc(dy);

            if (abs(dx) < 0.05)
                vel_x = 0;
            if (abs(dy) < 0.05)
                vel_y = 0;
        }
    }
    double cur_yaw =tf::getYaw(cur_pose.orientation);
    double dyaw;
    
    if (fix_angle>0 && cur_yaw <0 && (fix_angle- cur_yaw)> 3.14){
        dyaw = -(cur_yaw + 6.28-fix_angle);
    }
    else if (fix_angle<0 && cur_yaw >0 && (cur_yaw-fix_angle)> 3.14){
        dyaw = fix_angle  + 6.28- cur_yaw;
    }
    else{
        dyaw = fix_angle - cur_yaw;
    }
    ROS_INFO("cur_yaw: %f, dyaw %f ",cur_yaw, dyaw);
    vel_yaw = pid_yaw.calc(dyaw);
    if (abs(dyaw) < 0.1)
        vel_yaw = 0;

    msg_vel.linear.x = vel_x;
    msg_vel.linear.y = vel_y;
    msg_vel.angular.z = vel_yaw;
}

void RoboNav::setFixAngle(const geometry_msgs::Quaternion &qua)
{

    fix_angle = tf::getYaw(qua);
}

double Filter_ScanData(int index, const sensor_msgs::LaserScan::ConstPtr &sscan)
{
    int Cindex = index;
    double data = 8;
    int m = 0;
    for (int i = -23; i < 23; i++)
    {
        if (index + i < 0)
            Cindex = 360 + index + i;
        else if (index + i >= 360)
            Cindex = index + i - 360;
        else
            Cindex = index + i;
        if (sscan->ranges[Cindex] > 0.25 && sscan->ranges[Cindex] < data)
        {
            data = sscan->ranges[Cindex];
        }
    }

    return data;
}

void RoboNav::cb_scan(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    for (int i = 0; i < 4; i++)
    {
        int index = OFFSET + i * 90;
        int indexC = OFFSET + i * 90 + 45;
        obs_min[i][0] = Filter_ScanData(index, scan);
        obs_min[i][1] = Filter_ScanData(indexC, scan);
    }
    //ROS_INFO("min Front: %f, Left: %f  Behind: %f, Right: %f ", obs_min[0][0],obs_min[1][0],obs_min[2][0],obs_min[3][0]);
    //ROS_INFO("min corner Front: %f, Left: %f  Behind: %f, Right: %f ", obs_min[0][1],obs_min[1][1],obs_min[2][1],obs_min[3][1]);
}

geometry_msgs::Pose RoboNav::adjustlocalgoal()
{
    geometry_msgs::Pose local_goal;
    int local_goal_index = path[0];
    double local_goal_y = point_list.at<double>(local_goal_index, 0) * 1.0 / 100;
    double local_goal_x = point_list.at<double>(local_goal_index, 1) * 1.0 / 100;
    local_goal.position.y = point_list.at<double>(local_goal_index, 0) * 1.0 / 100;
    local_goal.position.x = point_list.at<double>(local_goal_index, 1) * 1.0 / 100;

    double dis_x = abs(local_goal_x - cur_pose.position.x);
    double dis_y = abs(local_goal_y - cur_pose.position.y);

    if (obs_min[0][0] < 0.34)
        pid_y.stop = true;
    else if (obs_min[0][0] < DEFFENCE) //front
    {
        pid_x.stop = false;
        pid_y.stop = false;
        local_goal.position.x = local_goal_x - 0.1;
    }
    else
    {
        pid_x.stop = false;
        pid_y.stop = false;
    }

    if (obs_min[0][1] < 0.45) //front-left
    {
        if (dis_x > dis_y)
            pid_x.stop = true;
        if (dis_x < dis_y)
            pid_y.stop = true;
    }
    else if (obs_min[0][1] < DEFF_CORNER)
    {
        pid_x.stop = false;
        pid_y.stop = false;
        local_goal.position.x = local_goal_x - 0.1;
        local_goal.position.y = local_goal_y - 0.1;
    }
    else
    {
        pid_x.stop = false;
        pid_y.stop = false;
    }

    if (obs_min[1][0] < 0.28)
        pid_x.stop = true;
    else if (obs_min[1][0] < DEFFENCE) //left
    {
        pid_x.stop = false;
        pid_y.stop = false;
        local_goal.position.y = local_goal_y - 0.1;
    }
    else
    {
        pid_x.stop = false;
        pid_y.stop = false;
    }

    if (obs_min[1][1] < 0.45) //left-back
    {
        if (dis_x > dis_y)
            pid_x.stop = true;
        if (dis_x < dis_y)
            pid_y.stop = true;
    }
    else if (obs_min[1][1] < DEFF_CORNER)
    {
        pid_x.stop = false;
        pid_y.stop = false;
        local_goal.position.x = local_goal_x + 0.1;
        local_goal.position.y = local_goal_y - 0.1;
    }
    else
    {
        pid_x.stop = false;
        pid_y.stop = false;
    }

    if (obs_min[2][0] < 0.34)
        pid_y.stop = true;
    else if (obs_min[2][0] < DEFFENCE) //back
    {
        pid_x.stop = false;
        pid_y.stop = true;
        local_goal.position.x = local_goal_x + 0.1;
    }
    else
    {
        pid_x.stop = false;
        pid_y.stop = false;
    }

    if (obs_min[2][1] < 0.45) //back-right
    {
        if (dis_x > dis_y)
            pid_x.stop = true;
        if (dis_x < dis_y)
            pid_y.stop = true;
    }
    else if (obs_min[2][1] < DEFF_CORNER)
    {
        pid_x.stop = false;
        pid_y.stop = false;
        local_goal.position.x = local_goal_x + 0.1;
        local_goal.position.y = local_goal_y + 0.1;
    }
    else
    {
        pid_x.stop = false;
        pid_y.stop = false;
    }

    if (obs_min[3][0] < 0.28)
        pid_x.stop = true;
    else if (obs_min[3][0] < DEFFENCE)
    {
        pid_x.stop = false;
        pid_y.stop = false;
        local_goal.position.y = local_goal_y + 0.1;
    }
    else
    {
        pid_x.stop = false;
        pid_y.stop = false;
    }

    if (obs_min[3][1] < 0.45) //right-front
    {
        if (dis_x > dis_y)
            pid_x.stop = true;
        if (dis_x < dis_y)
            pid_y.stop = true;
    }
    else if (obs_min[3][1] < DEFF_CORNER)
    {
        pid_x.stop = false;
        pid_y.stop = false;
        local_goal.position.x = local_goal_x - 0.1;
        local_goal.position.y = local_goal_y + 0.1;
    }
    else
    {
        pid_x.stop = false;
        pid_y.stop = false;
    }
    ROS_INFO("stop x: %d, y: %d", pid_x.stop, pid_y.stop);
    return local_goal;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robo_navigation");
    ros::NodeHandle nh;

    RoboNav robo_nav;
    robo_nav.init();
    ros::Subscriber cb_tar_pose = nh.subscribe("base/goal", 1, &RoboNav::cb_tar_pose, &robo_nav);
    ros::Subscriber cb_cur_pose = nh.subscribe("odom", 1, &RoboNav::cb_cur_pose, &robo_nav);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &RoboNav::cb_scan, &robo_nav);

    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher pub_state = nh.advertise<std_msgs::Bool>("nav_state", 1);

    ros::Rate rate(50);

    while (ros::ok())
    {
        
        geometry_msgs::Twist msg_vel;
        robo_nav.get_vel(msg_vel);
        //ROS_INFO("vel x: %f y:%f", msg_vel.linear.x, msg_vel.linear.y);
        //ROS_INFO("NUM: %d, SIZE: %ld", robo_nav.path[0], robo_nav.path.size());
        pub_vel.publish(msg_vel);
        pub_state.publish(robo_nav.state);
        ros::spinOnce();
        rate.sleep();
    }
}
