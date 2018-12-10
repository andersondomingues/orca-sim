#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

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

//ros references
ros::Publisher  pub_mpsoc_out;
ros::Subscriber pub_mpsoc_in;

udp_client* uclient;
udp_server* userver;

#define MOD_SKIP_PACKET 100
unsigned int packet_counter = 0;

//send several packets containing ranges from laser readings to 
//the mpsoc
void send_laser_to_mpsoc(const sensor_msgs::LaserScan::ConstPtr& msg){
	
	//make sure packet counter is always between 0 and MOD_SKIP_PACKET
	packet_counter = (packet_counter + 1) % MOD_SKIP_PACKET;
	int sent = 0;
	
	//ignore MOD_SKIP_PACKET packets
	if(packet_counter == 0){
		
		//buffes must be in the callback, otherwise
		//data will be overwritter by next iterations
		char buffer[UDP_BUFFER_LEN];

		//clean buffer
		for(int i = 0; i < UDP_BUFFER_LEN; i++) 
			buffer[i] = 0x00;

		//add noc header to the packet
		add_noc_header(buffer);
		
		//add laser info
		int j = 1;
		for(float i = msg->angle_min; i < msg->angle_max; i += msg->angle_increment){
			
			//ignoring outranged values
			if(msg->ranges[j] >= msg->range_min && msg->ranges[j] <= msg->range_max){
				
				//send 3xx ranges intead of 6xx
				if(j % 10 == 0){
				
					uint16_t index = j;
					uint16_t range = msg->ranges[j] * 100;					
					
					*(uint16_t*)&buffer[16] = index;
					*(uint16_t*)&buffer[18] = range;
					
					//ROS_INFO("%d = %d", index, range);
					
					uclient->send((const char*)buffer, UDP_BUFFER_LEN);				
					usleep(DELAY_BETWEEN_PACKETS);
					
					sent++;
				}
			}
			j++;
		}
		
		ROS_INFO("sent %d messages", sent);
		sent = 0;
	}
}

//msg.x is the index, msg.y is the range
void mpsoc_send_to_cmdvel(const geometry_msgs::Point msg){
	
	//generate angle from index
	ROS_INFO("received index is: %d", msg.x);
	
	//calculate twist
	
	//update cmdvel
	
}

//Callback that consumes message on the input topic and process
//information. Message is published back into output topic at
//the end of processing.
void laser_recv_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	send_laser_to_mpsoc(msg);
}

//Non-blocking function for receiveing packets from 
//the network
void* server_thread(void* v){
	
	//buffes must be in the callback, otherwise
	//data will be overwritter by next iterations
	char buffer[UDP_BUFFER_LEN];
	
	while(1){
	
		//recv packet from the network
		userver->recv(buffer, UDP_BUFFER_LEN);

		//pack into a ros structure
		geometry_msgs::Point p;
		
		uint16_t index = *(uint16_t*)&buffer[16];
		//index = (index >> 8) | (index << 8);
		
		uint16_t value = *(uint16_t*)&buffer[18];
		p.x = index;
		p.y = value;
		
		
		ROS_INFO("%d = %d", p.x, p.y);

		//calc new values for cmd_vel
		mpsoc_send_to_cmdvel(p);
	}
}


//Main routine: setup publishers and subscribers.
int main(int argc,char **argv)
{
	ros::init(argc,argv,"mpsoc_node");
	
	ros::NodeHandle n;
	
	//ROS_INFO();
	//subscribe and advertise to input and output topics. 
	pub_mpsoc_in  = n.subscribe("/hokuyo_laser", 10, laser_recv_callback); 
	pub_mpsoc_out = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	//initialize udp bridge
	const std::string& client_addr = MPSOC_ADDR;
	uclient = new udp_client(client_addr, MPSOC_PORT);

	const std::string& server_addr = ROSNODE_ADDR;
	userver = new udp_server(server_addr, ROSNODE_PORT);

	//start receiving thread
	pthread_t thread;
	pthread_create(&thread, NULL, server_thread, NULL);

	ros::spin(); 

	return 0;
}
