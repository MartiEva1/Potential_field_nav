#include <ros/ros.h>
#include <stdio.h>
#include <ctime>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "potential_fields_nav/DistanceMap.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

#define PI 3.141592

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> ActionServer;

using namespace potential_fields_nav;

static const std::string OPENCV_WINDOW = "Image";

DistanceMap* dmap;
ros::Publisher twist_pub;
bool success=false;
move_base_msgs::MoveBaseGoalConstPtr goal=NULL;

void distanceMap(const nav_msgs::OccupancyGridPtr& msg)
{
	int w = msg->info.width;
	int h = msg->info.height;
	float res = msg->info.resolution;
	
	double xo = msg->info.origin.position.x;
	double yo = msg->info.origin.position.y;
	
	geometry_msgs::Quaternion q = msg->info.origin.orientation;
	double yaw = std::atan2(2*(q.w * q.z + q.x * q.y), 1-2*(q.y * q.y + q.z * q.z));
	
	std::cout<<"MAPPA con: "<<std::endl;
	std::cout<<"larghezza: "<<w<<std::endl;
	std::cout<<"altezza: "<<h<<std::endl;
	std::cout<<"risoluzione: "<<res<<std::endl;
	std::cout<<"punto di origine: ("<<xo<<","<<yo<<")"<<std::endl;
	std::cout<<"yaw: "<<yaw<<std::endl;
	
	int* data=(int*)malloc((w*h)*sizeof(int));
	
	std::vector<int> occ;
	
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			data[i*w+j]=msg->data[i*w+j]; //100 => occupato; 0 => libera
			if(data[i*w+j] == 100)
				occ.push_back(i*w+j);
		}
	}
	
	dmap = new DistanceMap(data, w, h, res, xo, yo, yaw, occ);
	
	cv::Mat M(cv::Size(w,h), CV_8UC1, cv::Scalar(205));
	for(int i=0; i<h; i++)
	{
		for(int j=0; j<w; j++)
		{
			if(dmap->dmap[i*w+j].dist>0.5)
				M.at<uchar>(h-i,j) = 255;
			else if(dmap->dmap[i*w+j].dist>0 && dmap->dmap[i*w+j].dist<=0.5)
				M.at<uchar>(h-i,j) = dmap->dmap[i*w+j].dist*510;
		}
	}
	cv::imwrite("dmap.bmp", M);
	/*cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
	cv::resizeWindow(OPENCV_WINDOW, 800, 800);
	cv::imshow(OPENCV_WINDOW, M);
	cv::waitKey(1000);*/
}

void repulsiveField(const nav_msgs::OdometryPtr& msg)
{
	if(dmap==NULL)
	{
		std::cout<<"In attesa della mappa"<<std::endl;
		return;
	}
	
	srand(time(NULL));
	float ran=(float)rand()/RAND_MAX;
	
	int w = dmap->getWidth();
	int h = dmap->getHeight();
	float res = dmap->getResolution();
	
	//posizione del robot
	double xr = msg->pose.pose.position.x;
	double yr = msg->pose.pose.position.y;
	
	geometry_msgs::Quaternion q = msg->pose.pose.orientation;
	double theta_r = std::atan2(2*(q.w * q.z + q.x * q.y), 1-2*(q.y * q.y + q.z * q.z));
	
	//indici del robot nella distance map
	int ir, jr;
	ir = nearbyint(((yr - dmap->getOrigin().y))/res);
	jr = nearbyint((xr - dmap->getOrigin().x)/res);
	
	double dist = dmap->dmap[ir*w+jr].dist;
	double rot_obst_x=0, rot_obst_y=0, rot_obst=0;
	
	if(dmap->dmap[ir*w+jr].parent!=NULL && dmap->dmap[ir*w+jr].dist>0 && dmap->dmap[ir*w+jr].dist<0.1)
	{
		//indici dell'ostacolo più vicino
		int io = dmap->dmap[ir*w+jr].parent->r;
		int jo = dmap->dmap[ir*w+jr].parent->c;
		
		float dist_obst_x = (jr-jo)*res;
		float dist_obst_y = (ir-io)*res;
		
		//rotazione
		rot_obst = std::cos(theta_r)*dist_obst_x/dist - std::sin(theta_r)*dist_obst_y/dist;
	}
	else
	{
		rot_obst=0;
	}
	
	if(goal!=NULL)
	{
		double x_goal = goal->target_pose.pose.position.x;
		double y_goal = goal->target_pose.pose.position.y;
	
		int ig, jg;
		ig = nearbyint(((y_goal - dmap->getOrigin().y))/res);
		jg = nearbyint((x_goal - dmap->getOrigin().x)/res);


		float dist_goal_x = (jg-jr)*res;
		float dist_goal_y = (ig-ir)*res;
		
		if(std::fabs(dist_goal_x)<=0.20 && std::fabs(dist_goal_y)<=0.20)
		{
			success=true;
			geometry_msgs::Twist twist;
			twist.linear.x = 0;
			twist.angular.z = 0;
			twist_pub.publish(twist);
			goal=NULL;	
			return;
		}
		
		double k1=5, k2=1;
		
		double v, omega;
		
		v= k1 *(dist_goal_x*std::cos(theta_r) - dist_goal_y*std::sin(theta_r));
		
		
		if(rot_obst!=0)
		{
			omega= -k2*rot_obst;
			v = 0.3*v;
		}
		else
		{
			float x_goal_in_robot = (dist_goal_x*std::cos(theta_r) + dist_goal_y*std::sin(theta_r));
			float y_goal_in_robot = (-dist_goal_x*std::sin(theta_r) + dist_goal_y*std::cos(theta_r));
			
			omega = k2*std::atan2(y_goal_in_robot, x_goal_in_robot);
		}
		
		if(std::fabs(v)>5)
			v=v/std::fabs(v) * 5;
		
		if(std::fabs(omega)>2)
			omega=omega/std::fabs(omega)*2;
		
		geometry_msgs::Twist twist;
		twist.linear.x = v;
		twist.angular.z = omega;
		twist_pub.publish(twist);
	}
}

void execute(const move_base_msgs::MoveBaseGoalConstPtr& msg_goal, ActionServer* server)
{
	std::cout<<"Arrivato GOAL"<<std::endl;
	success=false;
	goal = msg_goal;
	ros::Time start = ros::Time::now();
	while(!success) 
	{
		if((ros::Time::now()-start)>=ros::Duration(80.0))
			break;
	}
	if(success)
	{
		std::cout<<"Obiettivo raggiunto... succeeded"<<std::endl;
		server->setSucceeded();
	}
	else
	{
		std::cout<<"Obiettivo non raggiunto... aborted"<<std::endl;
		server->setAborted();
	}
	
	success=false;
	goal=NULL;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "forceField_node");
	ros::NodeHandle nh;
  
	ros::Subscriber map_sub = nh.subscribe("/map", 1000, distanceMap);
	twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Subscriber pose_sub = nh.subscribe("/base_pose_ground_truth", 1000, repulsiveField);
	
	ActionServer server(nh, "move_base", boost::bind(&execute, _1, &server), false); 
	server.start();
 
	ros::spin();
}
