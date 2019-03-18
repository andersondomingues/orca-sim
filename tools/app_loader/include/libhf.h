#ifndef _LIBHF_H
#define _LIBHF_H

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <fstream>

#define NOC_PACKET_SIZE 64

#define LOADER_SERVICE_PORT 5001
#define LOADER_SERVICE_CHANNEL 5001

#define MICRO 1000000
#define RECV_BUFFER_LEN 128

void dump(char* _mem, uint32_t base, uint32_t length);

/*///////////////////////////////////////////////////////////////
  2 bytes   2 bytes           4 bytes  
 -----------------------------------------
 |    x    |     y   |    payload_len    |
 -----------------------------------------
	
  2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes   2 bytes       ....
 --------------------------------------------------------------------------------------------------
 |tgt_cpu  |payload  |src_cpu  |src_port |tgt_port |msg_size |seq      |channel  |  ... data ...  |
 --------------------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////*/
int32_t hf_send(uint16_t target_cpu, uint16_t target_port, int8_t *buf, uint16_t size, uint16_t channel);

#endif /*_LIBHF_H*/
