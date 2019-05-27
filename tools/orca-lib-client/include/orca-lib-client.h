#ifndef _ORCA_LIB_CLIENT_H
#define _ORCA_LIB_CLIENT_H

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <fstream>

#define NOC_WIDTH 4
#define NOC_HEIGHT 4

#define NOC_COLUMN(core_n)	((core_n) % NOC_WIDTH)
#define NOC_LINE(core_n)	((core_n) / NOC_WIDTH)

#define FLIT_SIZE 2

#define NOC_PACKET_SIZE_FLITS 64
#define NOC_PACKET_SIZE_BYTES (NOC_PACKET_SIZE_FLITS * FLIT_SIZE)

#define NOC_HEADER_SIZE_FLITS 8
#define NOC_HEADER_SIZE_BYTES (NOC_HEADER_SIZE_FLITS * FLIT_SIZE)

#define NOC_PAYLOAD_SIZE_FLITS (NOC_PACKET_SIZE_FLITS - NOC_HEADER_SIZE_FLITS)
#define NOC_PAYLOAD_SIZE_BYTES (NOC_PACKET_SIZE_BYTES - NOC_HEADER_SIZE_BYTES)

#define PKT_TARGET_CPU		0
#define PKT_PAYLOAD			1
#define PKT_SOURCE_CPU		2
#define PKT_SOURCE_PORT		3
#define PKT_TARGET_PORT		4
#define PKT_MSG_SIZE		5
#define PKT_SEQ				6
#define PKT_CHANNEL			7

#define LOADER_SERVICE_PORT 5000
#define LOADER_SERVICE_CHANNEL 5000

#define MICRO 1000000
#define RECV_BUFFER_LEN 128

void dump(char* _mem, uint32_t base, uint32_t length);

int32_t hf_send(uint16_t target_cpu, uint16_t target_port, 
	int8_t *buf, uint16_t size, uint16_t channel,
	std::string server_addr, uint32_t server_port);
	
int32_t hf_recv(uint16_t *source_cpu, uint16_t *source_port, 
	int8_t *buf, uint16_t *size, uint16_t *channel,
	std::string server_addr, uint32_t server_port);

#endif /*_ORCA_LIB_CLIENT_H */
