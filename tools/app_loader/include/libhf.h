#ifndef _LIBHF_H
#define _LIBHF_H

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <fstream>

#define NOC_PACKET_SIZE		64
#define PKT_HEADER_SIZE		8

#define PKT_TARGET_CPU		0
#define PKT_PAYLOAD			1
#define PKT_SOURCE_CPU		2
#define PKT_SOURCE_PORT		3
#define PKT_TARGET_PORT		4
#define PKT_MSG_SIZE		5
#define PKT_SEQ				6
#define PKT_CHANNEL			7

#define PAYLOAD_SIZE (NOC_PACKET_SIZE - PKT_HEADER_SIZE)

#define LOADER_SERVICE_PORT 5001
#define LOADER_SERVICE_CHANNEL 5001

#define MICRO 1000000
#define RECV_BUFFER_LEN 128

void dump(char* _mem, uint32_t base, uint32_t length);

/*///////////////////////////////////////////////////////////////
	
  2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes       ....
 --------------------------------------------------------------------------------------------------
 |tgt_cpu  |payload  |src_cpu  |src_port |tgt_port |msg_size |seq      |channel  |  ... data ...  |
 --------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////*/
int32_t hf_send(uint16_t target_cpu, uint16_t target_port, int8_t *buf, uint16_t size, uint16_t channel);

#endif /*_LIBHF_H*/
