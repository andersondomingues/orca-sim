#define TICK(X) clock_t X = clock()
#define TOCK(X) printf("time %s: %g sec.\n", (#X), (double)(clock() - (X)) / CLOCKS_PER_SEC)

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"

#include "../include/udp_client_server.h"
#include "../include/mpsoc_helper.h"

#include <iostream>

//Address of the mpsoc platform at the network
#define MPSOC_ADDR "127.0.0.1"
#define MPSOC_PORT 9999

//Address of current rosnode at the network.
//Must be set as localhost (loopback). Port can safely
//be changed as long as 
#define ROSNODE_ADDR "127.0.0.1"
#define ROSNODE_PORT 8888

//buffer size, not sure the effect cause by larger buffers
//but the waste of memory (50000 = 2msgs/min)
#define UDP_BUFFER_LEN 128
#define DELAY_BETWEEN_PACKETS 30000 

//hokuyo specific definitions
#define HOKUYO_ANGLE_MIN -2.268890
#define HOKUYO_ANGLE_MAX  2.268900
#define HOKUYO_ANGLE_INC  0.007101 

#define SECONDS_TO_MOVE_FORWARD 1
#define MOVE_SPEED 0.1
#define TURN_SPEED 0.05
#define TOLERANCE 0.02

//ros references
ros::Publisher  pub_mpsoc_out;
ros::Subscriber pub_mpsoc_in;
ros::Subscriber pub_odometry;

udp_client* uclient;
udp_server* userver;


//Main routine: setup publishers and subscribers.
int main(int argc,char **argv)
{
	ros::init(argc,argv,"mpsoc_node");
	ros::NodeHandle n;
	
	//initialize udp bridge
	const std::string& client_addr = MPSOC_ADDR;
	uclient = new udp_client(client_addr, MPSOC_PORT);

	const std::string& server_addr = ROSNODE_ADDR;
	userver = new udp_server(server_addr, ROSNODE_PORT);

	//arbitrary data
	uint16_t index = 0;
	uint16_t range = msg->ranges[j] * 100;					
	
	*(uint16_t*)&buffer[16] = index;
	*(uint16_t*)&buffer[18] = range;
	
	TICK(X)
	
	uclient->send((const char*)buffer, UDP_BUFFER_LEN);				
	userver->recv(buffer, UDP_BUFFER_LEN);

	TOCK(X)

	clock_t end = clock();
	double time_spent = (double)(end - begin) / CLOCKS_PER_SEC;

	ros::spin(); 

	return 0;
}
