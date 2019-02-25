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

//determine whether the vehicle is moving
bool move_is_locked = false;

//current pos
geometry_msgs::Pose2D curr_pose;

#define MOD_SKIP_PACKET 100
unsigned int packet_counter = 0;

//quarternion to eular angles (in radians already)
double toEulerAngle(double x, double y, double z, double w){
	double siny = +2.0 * 		(w * z + x * y);
	double cosy = +1.0 - 2.0 * 	(y * y + z * z);  
	return atan2(siny, cosy); 
}

//stores updated position 
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
		
	//update location
	curr_pose.x	= msg->pose.pose.position.x;
	curr_pose.y	= msg->pose.pose.position.y;

	//update angle
	curr_pose.theta = toEulerAngle(
		msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w //already in radians
	);	
}

//send several packets containing ranges from laser readings to 
//the mpsoc
void send_laser_to_mpsoc(const sensor_msgs::LaserScan::ConstPtr& msg){
	
	//make sure packet counter is always between 0 and MOD_SKIP_PACKET
	packet_counter = (packet_counter + 1) % MOD_SKIP_PACKET;
	int sent = 0;
	
	//ignore MOD_SKIP_PACKET packets
	if(packet_counter == 0){
		
		//fix moving and turning at the same time
		if(move_is_locked){
			ROS_INFO("scan dropped due previously unfinished scan");
			return;
		}else{
			move_is_locked = true;
		}
				
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
		for(float i = msg->angle_min; i < msg->angle_max && sent <= 52; i += msg->angle_increment){

			//ROS_INFO("min: %f, max: %f, inc: %f", msg->angle_min, msg->angle_max, msg->angle_increment);
	
			//ignoring outranged values
			if(msg->ranges[j] >= msg->range_min && msg->ranges[j] <= msg->range_max){
				
				//send 3xx ranges instead of 6xx
				if(j % 10 == 0){
				
					uint16_t index = j;
					uint16_t range = msg->ranges[j] * 100;					
					
					*(uint16_t*)&buffer[16] = index;
					*(uint16_t*)&buffer[18] = range;
					
					//ROS_INFO("%d = %d", index, range);
					//ROS_INFO();
					
					//ros::Time::now()
					
					uclient->send((const char*)buffer, UDP_BUFFER_LEN);				
					usleep(DELAY_BETWEEN_PACKETS);

					sent++;
				}
			}
			j++;
		}
					
		//when not enough rays to form a pack, complete the 
		//current pack with dummy rays, copied from the last
		//valid ray
		while(sent <= 52){
			uclient->send((const char*)buffer, UDP_BUFFER_LEN);	
			sent++;
		}
		
		ROS_INFO("sent %d messages", sent -1);
		sent = 0;
	}
}

void lock_move(){
	geometry_msgs::Twist t;
	t.linear.x  = 0;
	t.angular.z = 0;
	pub_mpsoc_out.publish(t);
	
	move_is_locked = true;
}

void unlock_move(){
	
	geometry_msgs::Twist t;
	t.linear.x  = -MOVE_SPEED;
	t.angular.z = 0;
	pub_mpsoc_out.publish(t);
	
	sleep(SECONDS_TO_MOVE_FORWARD);
	move_is_locked = false;
}
	


//msg.x is the index, msg.y is the range
void mpsoc_send_to_cmdvel(uint16_t index, uint16_t val){
	
	lock_move();
	
	ROS_INFO("received %d = %d", index, val);
	
	//generate angle from index
	float angle = HOKUYO_ANGLE_MIN;
	for(int i = 0; i < index; i++)
		angle += HOKUYO_ANGLE_INC;
		
	ROS_INFO("angle is %f radians", angle, val);	
	
	//calculate twist
	geometry_msgs::Twist t;
		
	//rotate until reach the target angle
	float distance = 0; 
	float pos_angle = 0; 
			
	//determine how much to rotate given the current angle
	float target_angle = 0;
	target_angle = (angle > 0) 
		? angle
		: M_PI + (M_PI - angle);

	while(1){
			
		//correct positive angle for curr_pos
		pos_angle = (curr_pose.theta > 0) 
			? curr_pose.theta 
			: M_PI + (M_PI - curr_pose.theta);
		
		distance = target_angle - pos_angle;
				
		t.angular.z = (distance < 0) ? TURN_SPEED : -TURN_SPEED;
		
		//ROS_INFO("t: %f, p: %f, d: %f", target_angle, pos_angle, distance);
				
		pub_mpsoc_out.publish(t);
		
		if(abs(distance) < TOLERANCE)
			break;
	}
	
	//make the vehicle move again
	unlock_move();
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

		//dump(buffer, 0, 100);

		//convert value from little to big endian
		uint16_t* buf16 = (uint16_t*)&buffer[16];
		
		uint16_t index = buf16[0];
		index = (index >> 8) | ((index & 0x0F)<< 8);
		
		uint16_t value = buf16[1];
		value = (value >> 8) | ((value & 0x0F)<< 8);
		
		//calc new values for cmd_vel
		mpsoc_send_to_cmdvel(index, value);
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
	pub_odometry  = n.subscribe("/odom", 10, odom_callback); 
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
