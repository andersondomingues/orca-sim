#include "udp_client_server.h"
#include <unistd.h>
#include <stdio.h>

#define RECV_BUFFER_LEN 128
#define MICRO 1000000

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


int main(int argc, char** argv){

	//connects to 8887 to send packets to the noc 
	const std::string& client_addr = "127.0.0.1";
	udp_client* uclient = new udp_client(client_addr, 9999);
	
	/*
	
  2 bytes   2 bytes           4 bytes  
 -----------------------------------------
 |    x    |     y   |    payload_len    |
 -----------------------------------------
	
  2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes       ....
 --------------------------------------------------------------------------------------------------
 |tgt_cpu  |payload  |src_cpu  |src_port |tgt_port |msg_size |seq      |channel  |  ... data ...  |
 --------------------------------------------------------------------------------------------------
	*/
	
	char* msg = new char[RECV_BUFFER_LEN];
	
	/* BEWARE THE ENDIANESSSSSS */
	
	msg[0] = 0x11;  //(1,1) is core #5
	msg[1] = 0x00; 

	msg[2] = 0x3e; 
	msg[3] = 0x00;  //length flit: 0x3e = 62 flits

	msg[4] = 0x00;  //payload
	msg[5] = 0x05;  //target_cpu (5)

	msg[6] = 0xe8;	//src_port (5000)
	msg[7] = 0x00;  //src_cpu (0,0)
	
	msg[8] = 0x88;  //msg_size
	msg[9] = 0x13;  //0x1388 = 5000 dec
	
	msg[10] = 0x64;
	msg[11] = 0x00;
	
	msg[12] = 0x01;
	msg[16] = 0x20;
	
	
	for(int i = 0; i < 20; i++)
		msg[17 + i] = 0x64;
	
	msg[40] = 0x0a;
	
	int sends = 0;

	while(1){

		uclient->send((const char*)msg, RECV_BUFFER_LEN);
		std::cout << "sent (" << sends << ")";
		dump(msg, 0, RECV_BUFFER_LEN); 
		std::cout << std::endl << std::endl;
		
		
		sends++;

		//1 000 000 microseconds = 1 seg
		usleep(1 * MICRO);
	}
}