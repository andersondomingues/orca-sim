#include "ros/ros.h"
#include "std_msgs/String.h"

#include "orca-lib.h"

#include <iostream>

//Address of the mpsoc platform at the network
#define MPSOC_ADDR "127.0.0.1"
#define MPSOC_PORT 9999

//Address of current rosnode at the network.
//Must be set as localhost (loopback). Port can safely
//be changed as long as 
#define ROSNODE_ADDR "127.0.0.1"
#define ROSNODE_PORT 8888

//ros references
ros::Publisher  orca_mpsoc_to_ros;
ros::Subscriber orca_ros_to_mpsoc;

//Non-blocking function to receive packets from mpsoc. This function
//will publish received topics in "orca_mpsoc_to_ros" topic. Multiple
//topics can be received, and the channel field can be used to discri-
//minate content.
void* recv_from_mpsoc(void* v){
	
	uint16_t source_cpu, source_port, data_size, channel;
	char msg[255];
	
	

	hf_recv_setup(ROSNODE_PORT);

	while(1){

		// Receive one message from the mpsoc through udp network 
		// using platform's client library. The function hf_recv is
		// similar to udp's recv_from. Please note that this is 
		// operation is blocking, and that is the reason we need an
		// auxiliary thread to handle packets. In the near future I should
		// provide an unblocking version of hf_recv. 
		//
		// Signature:
		//
		// int32_t hf_recv(
		// 		uint16_t *source_cpu, uint16_t *source_port, 
		// 		int8_t *buf, uint16_t *size, uint16_t channel)
		//
		// - source_cpu			node from the mpsoc that sent the message
		// - target_port		port in which the sender app is running
		// - data				data received
		// - size				size of received data, in bytes
		// - channel			tag. can be used to demux when receiving from multiple nodes
		//
		// Please note that there is no need to inform ip address and port
		// as this information is compiled within the platform (see Configuration.mk)
		int res = hf_recv(&source_cpu, &source_port, (int8_t*)&msg, &data_size, &channel);
		msg[data_size] = '\0';

		ROS_INFO("r: \"%s\", bytes=5d, cpu=%d, port=%d, channel=%d)", msg, data_size, source_cpu, source_port, channel);
		//ROS_INFO("r: \"%s\"", msg);
		
		//publish received data to the respective topic
		//orca_mpsoc_to_ros.publish(msg);
	}
}

//Callback that consumes message on the input topic and process
//information. Message is published back into output topic at
//the end of processing.
void orca_ros_to_mpsoc_callback(const std_msgs::String::ConstPtr& msg){
	
	char* c_str = new char[255];
	strcpy(c_str, msg->data.c_str());
	
	ROS_INFO("s: \"%s\"", c_str);
	
	hf_send(3, 5000, (int8_t*)c_str, strlen(c_str), 1000);
	hf_send(2, 5000, (int8_t*)c_str, strlen(c_str), 1000);
	hf_send(1, 5000, (int8_t*)c_str, strlen(c_str), 1000);
	
	// Forward the message to the mpsoc via udp using the 
	// platform's client library. The function hf_send is similar to 
	// the send function of udp with the addition of parameters 
	// to discriminate mpsoc nodes.
	//
	// Signature:
	//
	// int32_t hf_send(
	//	 	uint16_t target_cpu, uint16_t target_port, 
	//	 	int8_t *buf, uint16_t size, uint16_t channel,
	//	 	std::string mpsoc_addr, uint32_t mpsoc_port)
	//
	// - target_cpu			unique identifier of some mpsoc node
	// - target_port		port in which the app is running
	// - buf				pointer to the data to be sent
	// - size				size of data in bytes
	// - channel			tag. can be used to demux when receiving from multiple nodes
	// - mpsoc_addr			ip address of the mpsoc in the udp/ip network
	// - mpsoc_port			port of the mpsoc in the udp/ip network
	
}

//Main routine: setup publishers and subscribers. Data incoming from the 
//mpsoc will be published to the "orca_mpsoc_to_udp" topic, and all messages
//received on "orca_udp_to_mpsoc" will be pushed to mpsoc's udp adapter
int main(int argc,char **argv){

	ros::init(argc, argv, "example_orca_udp");
	ros::NodeHandle n;
	
	//publish to the "orca_mpsoc_to_ros" topic. All messages
	//in this topic come from the MPSoC
	orca_mpsoc_to_ros = n.advertise<std_msgs::String>("/orca_mpsoc_to_ros", 10);
	
	//subscribe to the "orca_ros_to_mpsoc topic". All messages in
	//this topic are forwarded to the MPSoC
	hf_send_setup(MPSOC_ADDR, MPSOC_PORT);
	orca_ros_to_mpsoc = n.subscribe("/orca_ros_to_mpsoc", 10, orca_ros_to_mpsoc_callback); 

	//start receiving thread
	pthread_t thread;
	pthread_create(&thread, NULL, recv_from_mpsoc, NULL);

	ros::spin(); 	

	return 0;
}
