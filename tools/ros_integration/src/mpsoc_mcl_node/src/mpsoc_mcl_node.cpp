#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"		// Motion
#include "nav_msgs/Odometry.h"			// Odometry
#include "tf/transform_datatypes.h"		// Transforms

#include "../include/udp_client_server.h"
#include "../include/mpsoc_helper.h"

#include <iostream>
#include <fstream>

//Address of the mpsoc platform at the network
#define MPSOC_ADDR "127.0.0.1"
#define MPSOC_PORT 9999
#define MPSOC_SIZE 16

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

//particle filter constants
#define PARTICLE_NUM	15
#define MAX_DEV			40		//janela de distribuicao das amostras
#define MAX_DEV_T		20		//Desvio maximo de theta
#define RESOLUTION		100		//100 pixels por metro
#define OFFSET_X		90
#define OFFSET_Y		150
#define OFFSET_T		40
#define NUM_SAMPLES		32

#define BMP_HEADER	1146
#define MAP_W		1792
#define MAP_H		1700

#define DISTANCE_VECTOR	1
#define MAP_VALUES		2
#define MAP_REQUEST		3
#define WEIGHT_REPONSE	4

#define chartou(A)		((char)(A) < 0 ? (A & 0x7f)+128 : (A))


//global map
unsigned char *map;

//ros references
ros::Subscriber odom_sub;
ros::Subscriber pub_mpsoc_in;
ros::Publisher  pub_mpsoc_out;

//definition of topic type; Include directive
//must be changed as well
typedef sensor_msgs::LaserScan topic_t ;

struct Particle {
	float w; //weight
	float x;
	float y;
	float theta;
};

struct Odometry {
	float x;
	float y;
	float theta;
};

//conection to udp network
udp_client* uclient;
udp_server* userver;


#define MOD_SKIP_PACKET 100
unsigned int packet_counter = 0;
unsigned int draw_counter = 0;

Odometry current_pose;
Particle particle_cloud[PARTICLE_NUM];

void init_cloud(struct Particle *p, float x, float y, float theta){
	int i;

	for (i = 0; i < PARTICLE_NUM; i++){
		p[i].w = 0.0;
		p[i].x = x + float(rand()%MAX_DEV*2 - MAX_DEV + OFFSET_X)/RESOLUTION;
		p[i].y = y + float(rand()%MAX_DEV*2 - MAX_DEV + OFFSET_Y)/RESOLUTION;
		p[i].theta = theta + float(rand()%MAX_DEV_T*2 - MAX_DEV_T + OFFSET_T)/RESOLUTION;

		if(p[i].theta > M_PI) p[i].theta = p[i].theta - 2*M_PI;
		if(p[i].theta < -M_PI) p[i].theta = p[i].theta + 2*M_PI;
	}
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
	nav_msgs::Odometry odom = *msg;
	geometry_msgs::Quaternion q = odom.pose.pose.orientation;

	current_pose.x = odom.pose.pose.position.x;
	current_pose.y = odom.pose.pose.position.y;
	current_pose.theta = tf::getYaw(q);
}

int check_bounds(int x, int y){
	if (x >= MAP_W || y >= MAP_H)
		return 0;
	return 1;
}

void draw_particle(unsigned char *map, Particle p){
	int xp = MAP_W/2 + p.x*RESOLUTION;
	int yp = MAP_H/2 + p.y*RESOLUTION;

	for(int y = yp-2; y <= yp+2; y++)
		for(int x = xp-2; x <= xp+2; x++)
			map[BMP_HEADER + y*MAP_W + x] = 0;

}

void draw_point(unsigned char *map, int xp, int yp){

	if(check_bounds(xp,yp))
		for(int y = yp-2; y <= yp+2; y++)
			for(int x = xp-2; x <= xp+2; x++)
				map[BMP_HEADER + y*MAP_W + x] = 128;

}


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
		unsigned char buffer[UDP_BUFFER_LEN];

		//clean buffer
		for(int i = 0; i < UDP_BUFFER_LEN; i++) 
			buffer[i] = 0x00;

		//add noc header to the packet
		//add_noc_header(buffer);
		
		//add laser info
		int j = 0;
		int index = 0;
		float last_angle = msg->angle_min;
		for(float i = msg->angle_min; i < msg->angle_max; i += msg->angle_increment){
			
			//ignoring outranged values
			if(msg->ranges[j] >= msg->range_min && msg->ranges[j] <= msg->range_max){
				
				//send 3xx ranges intead of 6xx
				if(j % 20 == 0){

					int range_i = msg->ranges[j] * 100;
				
					buffer[20+index*2] = range_i & 0x00ff;
					buffer[21+index*2] = (range_i & 0xff00) >> 8;
						
					//dump(buffer, 0, UDP_BUFFER_LEN);
					//printf("%d = %f, %f, sin = %f, cos = %f, buff = %d, %d\n", index, msg->ranges[j], i, sin(i), cos(i), buffer[21+index*2], buffer[20+index*2]);
					//printf("fix_val(%f),\n", i); //print angles

					index++;
					last_angle = i;
					
				}
			}
			j++;
			
		}

		init_cloud(particle_cloud, current_pose.x, current_pose.y, current_pose.theta);
		int index_ini = index;

		buffer[16] = DISTANCE_VECTOR; 

		for (int i = 1; i < MPSOC_SIZE; i++){
			index = index_ini;
			int xi = particle_cloud[i-1].x * 100;
			buffer[20+index*2] = xi & 0x00ff;
			buffer[21+index*2] = (xi & 0xff00) >> 8;
			index++;

			int yi = particle_cloud[i-1].y * 100;
			buffer[20+index*2] = yi & 0x00ff;
			buffer[21+index*2] = (yi & 0xff00) >> 8;
			index++;

			int theta_i = particle_cloud[i-1].theta * 100;
			buffer[20+index*2] = theta_i & 0x00ff;
			buffer[21+index*2] = (theta_i & 0xff00) >> 8;
			index++;

			add_noc_header(buffer, i);
			uclient->send((const char*)buffer, UDP_BUFFER_LEN);	
		}
		
		ROS_INFO("data sent");	
		//exit(0);
	}
	
	
}

//Non-blocking function for receiveing packets from 
//the network
void* mpsoc_out(void* v){
	
	//buffes must be in the callback, otherwise
	//data will be overwritter by next iterations
	char buffer[UDP_BUFFER_LEN];
	unsigned char send_buff[UDP_BUFFER_LEN];
	int x, y;
	unsigned char *output;
	
	while(1){
	
		//recv packet from the network
		userver->recv(buffer, UDP_BUFFER_LEN);
		
		char source = buffer[4];
		char service = buffer[17];

		if (service == MAP_REQUEST){

			if(source == 1 && draw_counter == 0){
				output = (unsigned char *) malloc(MAP_W*MAP_H+BMP_HEADER);
				memcpy(output, map, MAP_W*MAP_H+BMP_HEADER);
				draw_particle(output, particle_cloud[1]);
			}

			send_buff[16] = MAP_VALUES;

			for(int i = 0; i < NUM_SAMPLES/2; i++){
				x = (unsigned char)buffer[18+i*4] << 8 | (unsigned char)buffer[19+i*4];
				y = (unsigned char)buffer[20+i*4] << 8 | (unsigned char)buffer[21+i*4];
				if (check_bounds(x,y))
					send_buff[i+18] = map[BMP_HEADER + y*MAP_W + x];
				else
					send_buff[i+18] = 255;
				if(source == 1){
					draw_point(output, x, y);
					draw_counter++;
				}
			}
			usleep(25000);
			add_noc_header(send_buff, source);
			uclient->send((const char*)send_buff, UDP_BUFFER_LEN);

			if(source == 1 && draw_counter == 32){
				draw_counter = 0;
				ROS_INFO("writing image");
				//Write image for visualization
				FILE *fdo;
				if((fdo=fopen("output.bmp","w"))==NULL){
					std::cout << "Error in fopen: smooth_map.pgm" << " -> " << strerror(errno) << std::endl;
					exit (3);
				}
				if(fwrite(output,sizeof(unsigned char), MAP_W*MAP_H+BMP_HEADER, fdo) != MAP_W*MAP_H+BMP_HEADER) {fputs ("Writing error\n",stderr); exit (3);}
			}
		}

		if (service == WEIGHT_REPONSE){
			unsigned int prob;

			prob = (unsigned char)buffer[21] << 24 | (unsigned char)buffer[20] << 16 | 
				   (unsigned char)buffer[19] << 8  | (unsigned char)buffer[18];

			printf("wiehgt response %d = %d\n", source, prob);
		}
		
		//unpack data
		topic_t data;
		
		//publish through ros 
		pub_mpsoc_out.publish(data);

		//ROS_INFO("data recv'd");
	}
}

//Main routine: setup publishers and subscribers.
int main(int argc,char **argv)
{
	ros::init(argc,argv,"mpsoc_node");
	ros::NodeHandle n;

	//read map
	FILE *fd;
	if((fd=fopen("smooth_map.bmp","r"))==NULL){
		std::cout << "Error in fopen: smooth_map.bmp" << " -> " << strerror(errno) << std::endl;
		exit (3);
	}

	map = (unsigned char *) malloc(MAP_W*MAP_H+BMP_HEADER);
	if (fread(map, sizeof(unsigned char), MAP_W*MAP_H + BMP_HEADER, fd) != MAP_W*MAP_H+BMP_HEADER){
		std::cout << "Reading error." << std::endl;
	}

	//ROS_INFO();
	//subscribe and advertise to input and output topics. 
	odom_sub = n.subscribe("odom", 5, odom_callback);
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

	ros::spinOnce(); 
	usleep(200000);
	init_cloud(particle_cloud, current_pose.x, current_pose.y, current_pose.theta);

	ros::spin(); 

	return 0;
}


