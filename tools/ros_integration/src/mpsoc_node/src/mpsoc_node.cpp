#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h" // Hokuyo laser
#include "geometry_msgs/Twist.h"   // Motion
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"

#include <iostream>

using namespace geometry_msgs;

ros::Publisher  pub_mpsoc_out;
ros::Subscriber pub_mpsoc_in;
ros::NodeHandle n;

//Callbak that consumes message on the input topic and process
//information. Message is published back into output topic at
//the end of processing.
void mpsoc_in_callback(const nav_msgs::Odometry::ConstPtr& msg){

	ROS_INFO("chchchche..");
	
}

//Main routine: setup publishers and subscribers.
int main(int argc,char **argv)
{
	ros::init(argc,argv,"mpsoc_node");
	
	//subscribe and advertise to input and output topics. 
	pub_mpsoc_in  = n.subscribe("/mpsoc_in", 1, mpsoc_in_callback); 
	pub_mpsoc_out = n.advertise<geometry_msgs::Twist>("/mpsoc_out", 1);

	ros::spin(); 

	return 0;
}


