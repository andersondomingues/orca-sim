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

//messages ALWAYS have the same size
#define UDP_MAX_MESSAGE_SIZE 512

std::string __orca_send_ip_addr;
uint32_t    __orca_send_ip_port;
udp_client*  __orca_udp_client = nullptr;

std::string __orca_recv_ip_addr;
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
		hf_end_data_copy(&(msg[16]), (char*)&(buf[offset]), NOC_PAYLOAD_SIZE_BYTES);
		
		//adjust sequence number (zero recvs ok)
		msg[12] = i + 1;
		
		//send message chunk
		__orca_udp_client->send((const char*)msg, NOC_PACKET_SIZE_BYTES);			
	}

	return 0; //TODO: implement error messages
}

//treat endianess for the payload
//TODO: maybe we can treat the endiness for the whole packet
void hf_end_data_copy(char* target, char* source, size_t bytes){
	
	for(uint i = 0; i < bytes; i+=2){
			target[i] = source[i+1];
			target[i+1] = source[i];
	}
}

//we store only the port number as the server address, in this case,
//is always the same address as the one client process is running.
int32_t hf_recv_setup(std::string addr, uint32_t port){
	__orca_recv_ip_addr = addr;
	__orca_recv_ip_port = port;
	__orca_udp_server = new udp_server(__orca_recv_ip_addr, __orca_recv_ip_port);
	return 0;
}

//this function frees resources allocated previously through
//hf_recv_setup functions. It is necessary to dealocate if the
//port number changes
//TODO: support multiple connections
int32_t hf_recv_free(uint32_t port){
	return port;
}

//receive a message using settings from hf_recv_setup
int32_t hf_recv(uint16_t *source_cpu, uint16_t *source_port, 
	int8_t *buf, uint16_t *size, uint16_t* channel) //care channel vs. channel*
{
	
	uint16_t seq = 0, packet = 0, packets = 0;
	
	//recv from udp
	int8_t* buf_ptr = new int8_t[UDP_MAX_MESSAGE_SIZE];
	__orca_udp_server->recv((char*)buf_ptr, UDP_MAX_MESSAGE_SIZE);
	
	//copy next 102 bytes
	hf_end_data_copy((char*)buf, (char*)buf_ptr, UDP_MAX_MESSAGE_SIZE);

	
	uint16_t* bbuf = (uint16_t*)buf_ptr;

	*source_cpu = bbuf[PKT_SOURCE_CPU];
	*source_port = bbuf[PKT_SOURCE_PORT];
	*size = bbuf[PKT_MSG_SIZE];
	
	*channel = bbuf[PKT_CHANNEL];
	
	*source_port = bbuf[PKT_SOURCE_PORT];
	
	std::cout << "size = " << *size << std::endl;
	std::cout << "source_port = " << *source_port << std::endl;
	std::cout << "source_cpu = " << std::hex << *source_cpu << std::endl;
	
	//seq = buf[PKT_SEQ];
	
	hf_hexdump((char*)buf, 0, UDP_MAX_MESSAGE_SIZE);
		
	packets = (*size % NOC_PAYLOAD_SIZE_BYTES == 0) 
		? (*size / NOC_PAYLOAD_SIZE_BYTES) 
		: (*size / NOC_PAYLOAD_SIZE_BYTES + 1);

	if(packet > 1){
		printf("Warning: packet fragmentation not implemented yet. Possible loss of data.\n");
	}

	//!!!!! 
	memcpy(buf, &(buf[16]), *size);
	
	return seq + packets;
}


