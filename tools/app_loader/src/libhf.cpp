#include <cstring>

#include "libhf.h"
#include "udp_client_server.h"

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

/*///////////////////////////////////////////////////////////////////////////////////////////////////
  2 bytes   2 bytes           4 bytes  
 -----------------------------------------
 |    x    |     y   |    payload_len    |
 -----------------------------------------
	
  2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes       ....
 --------------------------------------------------------------------------------------------------
 |tgt_cpu  |payload  |src_cpu  |src_port |tgt_port |msg_size |seq      |channel  |  ... data ...  |
 --------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////*/
int32_t hf_send(uint16_t target_cpu, uint16_t target_port, int8_t *buf, uint16_t size, uint16_t channel)
{
	std::cout << target_cpu << target_port << buf[0] << size << channel << std::endl;

	char* msg = new char[RECV_BUFFER_LEN];
	
	//connects to 8887 to send packets to the noc 
	const std::string& client_addr = "127.0.0.1";
	udp_client* uclient = new udp_client(client_addr, 9999);
	
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

	msg[40] = 0x0a;		

	#define MAX_PAYLOAD_SIZE 102

	//calculate the number of packets
	//maximum of 102 flits of payload per packet (56 flits)
	uint32_t num_packets = size / MAX_PAYLOAD_SIZE;
	uint32_t seq_number = 0;

	//copy content into package (while remaining bytes)
	for(uint32_t i = 0; i < num_packets; i++){
		
		//copy next 102 (if any)
		memcpy(&(msg[17]), &(buf[seq_number * MAX_PAYLOAD_SIZE]), MAX_PAYLOAD_SIZE);
		
		//adjust sequence number
		msg[40] = seq_number;
		
		//send
		uclient->send((const char*)msg, RECV_BUFFER_LEN);
		dump(msg, 0, RECV_BUFFER_LEN);
		std::cout << std::endl << "sent " << num_packets << std::endl;
		
		num_packets--;
		seq_number++;
		
		sleep(1);
	}
	
	return 0;

}

