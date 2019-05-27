#include <cstring>

#include "orca-lib-client.h"
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
	
  2 bytes   2 bytes   2 bytes   2 bytes   4 bytes   4 bytes   4 bytes   2 bytes       ....
 --------------------------------------------------------------------------------------------------
 |tgt_cpu  |payload  |src_cpu  |src_port |tgt_port |msg_size |seq      |channel  |  ... data ...  |
 --------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////*/
int32_t hf_send(uint16_t target_cpu, uint16_t target_port, int8_t *buf, uint16_t size, uint16_t channel,
	std::string server_addr, uint32_t server_port)
{
	//std::cout << target_cpu << target_port << buf[0] << size << channel << server_addr << server_port << std::endl;

	char* msg = new char[NOC_PACKET_SIZE_BYTES];
	
	dump((char*)buf,  0, size);
		
	//connects to 8887 to send packets to the noc 
	//const std::string& client_addr = "127.0.0.1";
	udp_client* uclient = new udp_client(server_addr, server_port); //9999
			
	uint32_t x = NOC_COLUMN(target_cpu);
	uint32_t y = NOC_LINE(target_cpu);
		
	msg[0] = (x << 4) | y;	
	msg[1] = 0x00; 

	msg[2] = 0x3e; //length flit: 0x3e = 62 flits
	msg[3] = 0x00;  
	
	msg[4] = 0x00;  //src_cpu
	msg[5] = 0x00;  

	msg[6] = 0xe8;	//src_port (5000) ----- task
	msg[7] = 0x03;  

	msg[8] = (target_port & 0x000000FF);  //target port
	msg[9] = (target_port >> 8) & 0x000000FF;
	
	msg[10] = (size & 0x000000FF); //msg len
	msg[11] = (size & 0x0000FF00) >> 8; //TODO: maybe >> 8
	
	msg[12] = 0x01; //seq number
	msg[13] = 0x00; 
	
	msg[14] = (channel & 0x000000FF); //channel
	msg[15] = (channel & 0x0000FF00) >> 8;
	
	printf("Sending packet to core #%d (%d, %d) on port %d\n", 
		target_cpu, x, y, target_port);
		
	//calculate the number of packets
	//packet size = 64 flits = 128 bytes
	//header size =  8 flits =  32 bytes
	//payload size= 128-32 bytes = 96 bytes
	//maximum of 96 bytes of payload per packet (56 flits)
	//int payload_size = (PAYLOAD_SIZE * sizeof(uint16_t));
	
	uint32_t num_packets = size / NOC_PAYLOAD_SIZE_BYTES;
		
	//add one more packet to handle parts of the payload
	//not added in previous packets
	if(size % NOC_PAYLOAD_SIZE_BYTES != 0) 
		num_packets++;
		
	std::cout << "generating " << num_packets << " packets" <<std::endl;

	//copy content into package (while remaining bytes)
	for(uint32_t i = 0; i < num_packets; i++){
		
		//sleep(1);
		
		int offset = i * NOC_PAYLOAD_SIZE_BYTES;
		std::cout << std::endl << "offset: " << offset;
		
		//copy next 102 (if any)
		memcpy(&(msg[16]), &(buf[offset]), NOC_PAYLOAD_SIZE_BYTES);
		
		//adjust sequence number (zero recvs ok)
		msg[12] = i + 1;
		
		dump(msg, 0, NOC_PACKET_SIZE_BYTES);
		std::cout << "- - -" << std::flush;
		
		//send
		uclient->send((const char*)msg, RECV_BUFFER_LEN);			
	}
	
	std::cout << std::endl;	
	return 0;
}
