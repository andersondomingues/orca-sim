#ifndef _ORCA_NETWORK_BASE_H
#define _ORCA_NETWORK_BASE_H

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <fstream>

/**
 * hf_hexdump
 * @brief Prints data formated similarly to the hexdump tool
 * @param _mem Pointer to data chunk to print
 * @param base Byte to start printing from (starting position)
 * @param length Amount of bytes do print */
void hf_hexdump(char* _mem, uint32_t base, uint32_t length);

/**
 * hf_send_setup
 * @brief Set udp address and port for the mpsoc
 * @param server_port
 * @param server_addr
 * @return returns zero (0) if setup were sucefull */
int32_t hf_send_setup(std::string server_addr, uint32_t server_port);

/**
 * hf_send
 * @brief Sends a message to some process in the MPSoC
 * @param target_cpu Number of the node in which the receiver process runs
 * @param target_port Port in which the process is running
 * @param buf Pointer to the message to be sent (raw)
 * @param size Size of the message in bytes
 * @param channel Message type, in case of multiple messages to a same process
 * @return  returns zero (0) if operation occured succefully. */
int32_t hf_send(uint16_t target_cpu, uint16_t target_port,
	int8_t *buf, uint16_t size, uint16_t channel);

/**
 * hf_recv_open
 * @brief Enable the current process to receive packets from the mpsoc through udp
 * @param port Port number to which the udp connection will be instantiated
 * @return returns zero (0) if connection is succefull. */
int32_t hf_recv_setup(std::string addr, uint32_t port);

/**
 * hf_recv_close
 * @brief Free resources allocated from some udp connection instance, if any
 * @param port Port number to which the connection have been stablished before
 * @return returns zero (0) if the connection where succefully cleaned. */
int32_t hf_recv_free(uint32_t port);

/**
 * hf_receive
 * @brief Receive a message from the mpsoc
 * @param source_cpu Pointer to the variable to write the source node number
 * @param source_port Pointer to the variable to write the source node port
 * @param buf Pointer to write the message to
 * @param size Pointer to the variable to write the size of received message (in bytes)
 * @param channel Message type, if informed by the sender
 * @return */
int32_t hf_recv(uint16_t *source_cpu, uint16_t *source_port, 
	int8_t *buf, uint16_t *size, uint16_t *channel);

/**
 * hf_end_data_copy
 * @brief Apply endianess byte swap
 * @param target Ptr to the are to write results
 * @param source Data to have endianess applied
 * @param bytes Size of data to be swapped */
void hf_end_data_copy(char* target, char* source, size_t bytes);

#endif /*_ORCA_NETWORK_BASE_H */
