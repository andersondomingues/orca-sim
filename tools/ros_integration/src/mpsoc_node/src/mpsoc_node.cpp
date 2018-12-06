#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"   // Motion

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
//but the waste of memory
#define UDP_BUFFER_LEN 128

//laser specific definition
#define HOKUYO_NUM_RANGES 


//ros references
ros::Publisher  pub_mpsoc_out;
ros::Subscriber pub_mpsoc_in;

//definition of topic type; Include directive
//must be changed as well
typedef sensor_msgs::LaserScan topic_t ;

//conection to udp network
udp_client* uclient;
udp_server* userver;


#define MOD_SKIP_PACKET 100
unsigned int packet_counter = 0;

//Callback that consumes message on the input topic and process
//information. Message is published back into output topic at
//the end of processing.
void mpsoc_in_callback(const topic_t::ConstPtr& msg){

	//make sure packet counter is always between 0 and MOD_SKIP_PACKET
	packet_counter = (packet_counter + 1) % MOD_SKIP_PACKET;
	
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
				if(j % 2 == 0){
				
					buffer[20] = msg->ranges[j] * 100;
						
						
					uclient->send((const char*)buffer, UDP_BUFFER_LEN);		
					printf("%d = %f, inc = %f\n", j, i, msg->angle_increment);
				}
			}
			j++;
			
		}
		
		ROS_INFO("data sent");	
		
	}
}

//Non-blocking function for receiveing packets from 
//the network
void* mpsoc_out(void* v){
	
	//buffes must be in the callback, otherwise
	//data will be overwritter by next iterations
	char buffer[UDP_BUFFER_LEN];
	
	while(1){
	
		//recv packet from the network
		userver->recv(buffer, UDP_BUFFER_LEN);
		
		char* start_addr = &buffer[20];
		
		//unpack data
		topic_t data;
		
		//publish through ros 
		pub_mpsoc_out.publish(data);
		
		ROS_INFO("data recv'd");
	}
}


//Main routine: setup publishers and subscribers.
int main(int argc,char **argv)
{
	ros::init(argc,argv,"mpsoc_node");
	
	ros::NodeHandle n;
	
	//ROS_INFO();
	//subscribe and advertise to input and output topics. 
	pub_mpsoc_in  = n.subscribe("/hokuyo_laser", 10, mpsoc_in_callback); 
	pub_mpsoc_out = n.advertise<topic_t>("/mpsoc_out", 10);

	//initialize udp bridge
	const std::string& client_addr = MPSOC_ADDR;
	uclient = new udp_client(client_addr, MPSOC_PORT);

	const std::string& server_addr = ROSNODE_ADDR;
	userver = new udp_server(server_addr, ROSNODE_PORT);

	//start receiving thread
	pthread_t thread;
	pthread_create(&thread, NULL, mpsoc_out, NULL);

	ros::spin(); 

	return 0;
}


