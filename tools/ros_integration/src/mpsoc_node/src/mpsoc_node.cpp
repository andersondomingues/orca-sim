#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"   // Motion

#include "../include/udp_client_server.h"

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
//but the waste of memory
#define UDP_BUFFER_LEN 200

//ros references
ros::Publisher  pub_mpsoc_out;
ros::Subscriber pub_mpsoc_in;

//conection to udp network
udp_client* uclient;
udp_server* userver;


//function that add noc headers to the buffer and 
//return the initial address of the payload
char* add_noc_headers(char* buffer){
	
	//care the endianess!
	buffer[0] = 0x11;  //(1,1) is core #5
	buffer[1] = 0x00;  //reserved

	//length flit: 0x3e = 62 flits (124 bytes)
	buffer[2] = 0x3e;  
	buffer[3] = 0x00;  

	buffer[4] = 0x00;  //payload
	buffer[5] = 0x05;  //target_cpu (5)

	buffer[6] = 0x00;  //src_port (unused,)
	buffer[7] = 0x00;  //src_cpu  (0,0 is the gateway addr)
	
	//target_port (0x1388 = 5000 dec)
	buffer[8] = 0x88;  
	buffer[9] = 0x13;  
	
	//TODO: double check headers 
	return &buffer[20];
}

char* remove_noc_headers(char* buffer){
	return &buffer[20];
}

//Callback that consumes message on the input topic and process
//information. Message is published back into output topic at
//the end of processing.
void mpsoc_in_callback(const geometry_msgs::Vector3::ConstPtr& msg){

	//buffes must be in the callback, otherwise
	//data will be overwritter by next iterations
	char buffer[UDP_BUFFER_LEN];

	//add noc headers 
	//char* start_addr = add_noc_headers(buffer);
	
	//add payload (TODO: serialization)
	//*(float*)start_addr = msg.getX();
	//start_addr += 8;
	
	//*(float*)start_addr = msg.py;
	//start_addr += sizeof(msg.py);
	
	//*(float*)start_addr = msg.pz;
	//start_addr += sizeof(msg.pz);
	
	//send via udp
	uclient->send((const char*)buffer, UDP_BUFFER_LEN);
	
	//wait for response 
	userver->recv(buffer, UDP_BUFFER_LEN);
	
	//do whatever has to be done with the data
	//start_addr = remove_noc_headers(buffer);
	ROS_INFO("lalala finished");		
}

//Main routine: setup publishers and subscribers.
int main(int argc,char **argv)
{
	ros::init(argc,argv,"mpsoc_node");
	
	ros::NodeHandle n;
	
	//subscribe and advertise to input and output topics. 
	pub_mpsoc_in  = n.subscribe("/mpsoc_in", 1, mpsoc_in_callback); 
	pub_mpsoc_out = n.advertise<geometry_msgs::Vector3>("/mpsoc_out", 1);

	//initialize udp bridge
	const std::string& client_addr = MPSOC_ADDR;
	uclient = new udp_client(client_addr, MPSOC_PORT);

	const std::string& server_addr = ROSNODE_ADDR;
	userver = new udp_server(server_addr, ROSNODE_PORT);

	ros::spin(); 

	return 0;
}


