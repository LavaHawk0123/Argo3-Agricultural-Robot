#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalID.h"
#include <cmath>

class Cancel
{
	public:
		float goal_x;
		float goal_y;
		float odom_x;
		float odom_y;
		float dist;
		int status;
		bool flag;
		ros::Publisher pub;
		//ros::Subscriber sub_stat;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_goal;
		ros::Subscriber sub;

	Cancel(ros::NodeHandle *n)
	{
		goal_x =0;
		goal_y=0;
		odom_x =0;
		odom_y=0;
		dist =0;
		status=0;
		flag = false;
		std::cout<<"Entered Constructor";
		pub = n->advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
		//sub_stat = n->subscribe("/move_base/status",1000, &Cancel::getstat,this);
		sub_goal = n->subscribe("/move_base/goal",1000, &Cancel::getgoal,this);
		sub_odom = n->subscribe("/odom",1000,&Cancel::getodom,this);
		std::cout<<"\n In init : "<< goal_x<<"\n";
	}
	//void getgoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);
	//void getodom(const nav_msgs::Odometry::ConstPtr& odom);
	//void getstat(const actionlib_msgs::GoalStatusArray::ConstPtr& stat);
	//bool check();


void getgoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg)
{
	std::cout<<"\n Entered Callback \n";
	goal_x = msg->goal.target_pose.pose.position.x;
	std::cout<<"\n Got x coordinate \n"<<goal_x;
	goal_y = msg->goal.target_pose.pose.position.y;
	std::cout<<"\n Got y coordinate \n"<<goal_y;
	std::cout<<"\n Received new goal "<< goal_x << goal_y;
	

}

void getodom(const nav_msgs::Odometry::ConstPtr& odom)
{
	if(odom)
	{
		odom_x= odom->pose.pose.position.x;
		odom_y= odom->pose.pose.position.y;
		dist = sqrt(pow((odom_x-goal_x),2)+pow((odom_y-goal_y),2));
		std::cout<<"Distance from goal :"<<dist<<"\n";
		//std::cout<<"\n Status: "<<status;
		if(dist<0.2 and goal_x!= 0 and goal_x!=11.63 and goal_x!=27.48)
		{
			actionlib_msgs::GoalID goal;
			pub.publish(goal);
			std::cout<<"\n Reached Goal, Sent cancel command";
			goal_x=0;
			goal_y=0;
		}
		else if(dist<0.2 and (goal_x==11.63 or goal_x==27.48))
		{
			actionlib_msgs::GoalID goal;
			pub.publish(goal);
			std::cout<<"\n Reached Goal, Sent cancel command";
			goal_x=0;
			goal_y=0;
		}

	}
	else{
		std::cout<<"\n No odom recieved";
	}
}



};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Cancel");
    ros::NodeHandle n;
    Cancel ob=Cancel(&n);
    ros::spin();
    return 0;
}
