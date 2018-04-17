#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class RoboSentGoal{
	public:
		ros::NodeHandle* pnh;
		MoveBaseClient* ac; 
		ros::Publisher cancle_pub;
        move_base_msgs::MoveBaseGoal goal; 
        int goal_update;
		
		RoboSentGoal(){
			pnh=new ros::NodeHandle("");
			cancle_pub= pnh->advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
			ac=new MoveBaseClient("move_base", true);
            goal_update=0;
		};
		void cb_pose(const geometry_msgs::PoseConstPtr& msg);
};

void RoboSentGoal::cb_pose(const geometry_msgs::PoseConstPtr& msg){
	
	goal.target_pose.header.frame_id = "odom"; 
    goal.target_pose.header.stamp = ros::Time::now(); 
    
    goal.target_pose.pose.position= msg->position;
    goal.target_pose.pose.orientation=msg->orientation;
	goal_update=1;
    ROS_INFO("*******************Get goal*************");
}


int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "robo_sent_goal"); 
	ros::NodeHandle nh;
	
    MoveBaseClient ac("move_base", true);
    
	RoboSentGoal mp;
    
	ros::Subscriber sub_pose = nh.subscribe("base/goal", 1, &RoboSentGoal::cb_pose, &mp);
    ros::Rate rate(10);
    int k=0;
    while (ros::ok()){

        if ( mp.goal_update==1 && ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("***********************Set Goal*****************************");
            ac.sendGoal(mp.goal);
            mp.goal_update=0;
        }
        ros::spinOnce();
        rate.sleep();
    }

}