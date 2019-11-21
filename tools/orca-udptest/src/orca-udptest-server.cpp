#include <orca-udp-test.h>

#include <orca-network-base.h>
#include <orca-udp-client-server.h>

#include <unistd.h>
#include <stdio.h>

#include <chrono>

#include <iostream>
#include <fstream>
#include <cstring>

#define SERVICE_PORTNUM 5000
#define SERVICE_CHANNEL 5000
#define SERVICE_UDPADDR "127.0.0.1"
#define SERVICE_UDPPORT 9999

/* entry-point */
int main(int argc, char** argv){ 

	std::cout <<
		"_____________________________________________" << std::endl << 
		"                                             " << std::endl << 
		"          --  ORCA-UDP-TEST tool  --         " << std::endl << 
		"                                             " << std::endl << 
		"   http://github.com/ursa                    " << std::endl << 
		"   http://github.com/orca-mpsoc              " << std::endl;

	std::cout <<
		"_____________________________________________" << std::endl <<
		"                                             " << std::endl <<
		" build " << __DATE__ << ":" << __TIME__ << " " << std::endl;

	std::cout <<
		"_____________________________________________" << std::endl <<
		"                                             " << std::endl <<
		" USAGE:                                      " << std::endl <<
		" orca-udp-test <ip> <port> <core> <dp> <dly> " << std::endl <<
		"                                             " << std::endl <<
		" ip:     IP address of emulated interface    " << std::endl <<
		" port:   UDP port of emulated interface      " << std::endl <<
		" core:   Id of the target core cpu           " << std::endl <<
		" dp:     Destination port of target task     " << std::endl <<
		" dly:    Delay between packets (ms)          " << std::endl <<
		"_____________________________________________" << std::endl;

	//argv[0] has exe name
	if(argc < 6){
		std::cout << " BAD invokation, see USAGE" << std::endl;
		return 0;
	}

	unsigned int big_number = 0;
	
	std::string  server_ip   = std::string(argv[1]);
	unsigned int server_port = atoi(argv[2]);

	uint16_t core_num  = atoi(argv[3]);
	uint16_t core_port = atoi(argv[4]);
	
	unsigned int delay = atoi(argv[5]);

	hf_send_setup(server_ip, server_port);

	//keeps running forever
	while(1){
		hf_send(core_num, core_port, (int8_t*)&(big_number), sizeof(big_number), 12345);
		std::cout << "sent " << big_number << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(delay));
	}
	return 0;
}
