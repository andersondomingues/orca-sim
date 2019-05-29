#include <cstring>

#include "orca-network-base.h"
#include "orca-udp-client-server.h"

//as defined for orca (see hermes noc documentation)
#define FLIT_SIZE 2

//message format define in hellfireos
#define NOC_PACKET_SIZE_FLITS 64
#define NOC_PACKET_SIZE_BYTES (NOC_PACKET_SIZE_FLITS * FLIT_SIZE)

#define NOC_HEADER_SIZE_FLITS 8
#define NOC_HEADER_SIZE_BYTES (NOC_HEADER_SIZE_FLITS * FLIT_SIZE)

#define NOC_PAYLOAD_SIZE_FLITS (NOC_PACKET_SIZE_FLITS - NOC_HEADER_SIZE_FLITS)
#define NOC_PAYLOAD_SIZE_BYTES (NOC_PACKET_SIZE_BYTES - NOC_HEADER_SIZE_BYTES)

//header codes, define in noc.h  (hellfireos)
#define PKT_TARGET_CPU		0
#define PKT_PAYLOAD			1
#define PKT_SOURCE_CPU		2
#define PKT_SOURCE_PORT		3
#define PKT_TARGET_PORT		4
#define PKT_MSG_SIZE		5
#define PKT_SEQ				6
#define PKT_CHANNEL			7

//as defined in noc.h (hellfireos)
#define NOC_COLUMN(core_n)	((core_n) % ORCA_NOC_WIDTH)
#define NOC_LINE(core_n)	((core_n) / ORCA_NOC_WIDTH)

std::string __orca_send_ip_addr;
uint32_t    __orca_send_ip_port;
udp_client*  __orca_udp_client = nullptr;

std::string __orca_recv_ip_addr = "127.0.0.1";
uint32_t    __orca_recv_ip_port;
udp_server* __orca_udp_server = nullptr;

// Similar function is implemented into hellfire, but for 
// a fix when printing negative values. We define such a 
// function here so that one may compare packets in both
// the sides of communication at the application level.
void hf_hexdump(char* _mem, uint32_t base, uint32_t length){
	
	uint32_t k, l;
	
	//mask is necessary to correct a bug(?) 
	//when printing negative values.
	uint32_t mask = 0x000000FF; 
	int8_t ch;
	
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

// hf_send setup stores the values for addr and port
// of the mpsoc in the network, so we can use these 
// values while sending messages. We implement the
// setup separatelly from the send function as the 
// underlying networking class requires address and
// port to be informed while instantantiating the
// udp client object.
int32_t hf_send_setup(std::string server_addr, uint32_t server_port){
	__orca_send_ip_addr = server_addr;
	__orca_send_ip_port = server_port;
	__orca_udp_client = new udp_client(server_addr, server_port);
	return 0; //TODO: implement error messages
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
int32_t hf_send(uint16_t target_cpu, uint16_t target_port,
	int8_t *buf, uint16_t size, uint16_t channel)
{
	char* msg = new char[NOC_PACKET_SIZE_BYTES];
			
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
		
	//copy content into package (while remaining bytes)
	for(uint32_t i = 0; i < num_packets; i++){
			
		int offset = i * NOC_PAYLOAD_SIZE_BYTES;
		
		//copy next 102 bytes
		memcpy(&(msg[16]), &(buf[offset]), NOC_PAYLOAD_SIZE_BYTES);
		
		//adjust sequence number (zero recvs ok)
		msg[12] = i + 1;
		
		//send message chunk
		__orca_udp_client->send((const char*)msg, NOC_PACKET_SIZE_BYTES);			
	}

	return 0; //TODO: implement error messages
}


int32_t hf_recv_setup(uint32_t port){
	__orca_recv_ip_port = port;
	__orca_udp_server = new udp_server(__orca_recv_ip_addr, __orca_recv_ip_port);
	return 0;
}

int32_t hf_recv_free(uint32_t port){
	return port;
}

int32_t hf_recv(uint16_t *source_cpu, uint16_t *source_port, 
	int8_t *buf, uint16_t *size, uint16_t *channel){
	
	return *source_cpu + *source_port + buf[0] + *size + *channel;		
}


