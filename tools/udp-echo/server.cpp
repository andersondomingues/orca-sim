#include "udp_client_server.h"
#include <iostream>
#include <unistd.h>
#include <stdio.h>

#define RECV_BUFFER_LEN 128
#define MICRO 1000000

int main(int argc, char** argv){

	//connects to 8887 to send packets to the noc 
	const std::string& server_addr = "127.0.0.1";
	udp_server* userver = new udp_server(server_addr, 8888);
	
	//message
	char* msg = new char[RECV_BUFFER_LEN];
	msg[0] = 0;
	msg[1] = 1;
	
	int recs = 0;

	while(1){
		userver->recv(msg, RECV_BUFFER_LEN);
		
		std::cout << "recv'd (" << recs << ")" << std::endl;
				
		recs++;
	}
}