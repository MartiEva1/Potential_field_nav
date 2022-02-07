#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>

#define GX -20.875
#define GY 21.5 //GY 20.920
#define GZ 0.998
#define GW -0.067

#define RVIZ 1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
MoveBaseClient* ac;

void setGoalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
	move_base_msgs::MoveBaseGoal goal;
		
	goal.target_pose.header.frame_id="base_link";
	goal.target_pose.header.stamp=ros::Time::now();
	goal.target_pose.pose.position.x = msg->pose.position.x;
	goal.target_pose.pose.position.y = msg->pose.position.y;
	goal.target_pose.pose.orientation.z = msg->pose.position.z;
	goal.target_pose.pose.orientation.w = msg->pose.orientation.w;

	ROS_INFO("Mando il goal");
		
	ac->sendGoal(goal);
		
	ac->waitForResult(ros::Duration(80.0));
		
	if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, obiettivo raggiunto!!!");
	else
		ROS_INFO("Obiettivo non raggiunto!!");
}

int main(int argc, char** argv)
{	
	double gx, gy, gz, gw;
	
	ros::init(argc, argv, "client_node");
	ros::NodeHandle nh;
	
	ac= new MoveBaseClient("move_base", true);
	
	while(!ac->waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Aspettando che il server si connetta");
	}
		
	ROS_INFO("server partito, manda un goal"); 
	
	if(RVIZ==1)
	{
		ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 1000, setGoalCallback);
		ros::spin();
	}
	else
	{
		if(argc==5)
		{
			gx=atof(argv[1]);
			gy=atof(argv[2]);
			gz=atof(argv[3]);
			gw=atof(argv[4]);
		}
		else
		{
			gx=GX;
			gy=GY;
			gz=GZ;
			gw=GW;
		}
		
		
		move_base_msgs::MoveBaseGoal goal;
		
		goal.target_pose.header.frame_id="base_link";
		goal.target_pose.header.stamp=ros::Time::now();
		goal.target_pose.pose.position.x = gx;
		goal.target_pose.pose.position.y = gy;
		goal.target_pose.pose.orientation.z = gz;
		goal.target_pose.pose.orientation.w = gw;

		ROS_INFO("Mando il goal");
		
		ac->sendGoal(goal);
		
		ac->waitForResult(ros::Duration(80.0));
		
		if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, obiettivo raggiunto!!!");
		else
			ROS_INFO("Obiettivo non raggiunto!!");
	}
	return 0;
}
