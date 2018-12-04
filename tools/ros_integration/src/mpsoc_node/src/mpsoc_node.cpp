#include "ros/ros.h"
#include "geometry_msgs/Point.h"   // Motion

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
#define UDP_BUFFER_LEN 128

//ros references
ros::Publisher  pub_mpsoc_out;
ros::Subscriber pub_mpsoc_in;

//definition of topic type; Include directive
//must be changed as well
typedef geometry_msgs::Point topic_t ;

//conection to udp network
udp_client* uclient;
udp_server* userver;

//shorthand print
void dump(char* _mem, uint32_t base, uint32_t length){
	uint32_t k, l;
	
	//mask is necessary to correct a bug(?) when printing
	//negative hexas.
	uint32_t mask = 0x000000FF; 
	int8_t ch;
	
	//uint32_t* memptr = (uint32_t*)_mem;
	//uint32_t  len = _length / 4;
	for(k = 0; k < length; k += 16){
		printf("\n%08x ", base + k);
		for(l = 0; l < 16; l++){
			printf("%02x ", _mem[k + l] & mask );
			if (l == 7) putchar(' ');
		}
		printf(" |");
		for(l = 0; l < 16; l++){
			ch = _mem[k + l];
			if ((ch >= 32) && (ch <= 126))
				putchar(ch);
			else
				putchar('.');
		}
		putchar('|');
	}
}

//serialize
void* pack_data(char* addr, const topic_t::ConstPtr& msg){

	char* ptr = addr;
	
	//add x val
	uint32_t x_val = msg->x * 100000;
	*(uint32_t*)ptr = x_val;
	
	//inc 4 bytes
	ptr += 4;
	
	//add x val
	uint32_t y_val = msg->y * 100000;
	*(uint32_t*)ptr = y_val;
	
	ptr += 4;
	
	//add x val
	uint32_t z_val = msg->z * 100000;
	*(uint32_t*)ptr = z_val;

}

//unserialize
topic_t unpack_data(char* addr){
	
	topic_t t;
	
	
	ROS_INFO("unknown type, cannot unpack data");
	
}

//Callback that consumes message on the input topic and process
//information. Message is published back into output topic at
//the end of processing.
void mpsoc_in_callback(const topic_t::ConstPtr& msg){

	//buffes must be in the callback, otherwise
	//data will be overwritter by next iterations
	char buffer[UDP_BUFFER_LEN];

	for(int i = 0; i < UDP_BUFFER_LEN; i++)
		buffer[i] = 0x00;

	//add noc headers 
	buffer[0] = 0x11;  //(1,1) is core #5
	buffer[1] = 0x00; 

	buffer[2] = 0x3e; 
	buffer[3] = 0x00;  //length flit: 0x3e = 62 flits

	buffer[4] = 0x00;  //payload
	buffer[5] = 0x05;  //target_cpu (5)

	buffer[6] = 0xe8;	//src_port (5000)
	buffer[7] = 0x00;  //src_cpu (0,0)
	
	buffer[8] = 0x88;  //msg_size
	buffer[9] = 0x13;  //0x1388 = 5000 dec
	
	buffer[10] = 0x64;
	buffer[11] = 0x00;
	
	buffer[12] = 0x01;
	buffer[16] = 0x20;
	
	for(int i = 0; i < 20; i++)
		buffer[17 + i] = 0x63;
	
	buffer[40] = 0x0a; // \n
	
	//pack data
	pack_data(&buffer[20], msg);
	
	//send via udp
	uclient->send((const char*)buffer, UDP_BUFFER_LEN);
	
	dump(buffer, 0, UDP_BUFFER_LEN);
	printf("\n");
	
	//do whatever has to be done with the data
	//start_addr = remove_noc_headers(buffer);
	ROS_INFO("data sent");
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
		data = unpack_data(start_addr);
		
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
	pub_mpsoc_in  = n.subscribe("/mpsoc_in", 1, mpsoc_in_callback); 
	pub_mpsoc_out = n.advertise<topic_t>("/mpsoc_out", 1);

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


