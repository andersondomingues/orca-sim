#include "udp_client_server.h"
#include <unistd.h>
#include <stdio.h>

#define RECV_BUFFER_LEN 128
#define MICRO 1000000

int main(int argc, char** argv){

	//connects to 8887 to send packets to the noc 
	const std::string& client_addr = "127.0.0.1";
	udp_client* uclient = new udp_client(client_addr, 9999);
	
	//message
	char* msg = new char[RECV_BUFFER_LEN];
	msg[0] = 1;
	msg[1] = 1;
	msg[3] = 0x3e;
	
	int sends = 0;

	while(1){

		uclient->send((const char*)msg, RECV_BUFFER_LEN);
		std::cout << "sent (" << sends << ")" << std::endl;
		sends++;

		//1 000 000 microseconds = 1 seg
		usleep(1 * MICRO);
	}
}