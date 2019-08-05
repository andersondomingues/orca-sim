/** 
 * This file is part of project URSA. More information on the project
 * can be found at 
 *
 * URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
 *
 * Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. **/
#ifndef __TNETSOCKET_H
#define __TNETSOCKET_H

//std API
#include <iostream>
#include <pthread.h>

//simulator API
#include <TimedModel.h>

//dependency models
#include <UBuffer.h>
#include <UMemory.h>
#include <UComm.h>
#include <TNetif.h>

typedef uint16_t FlitType;

enum class TNetSocketRecvState{ READY, DATA_IN, FLUSH};

enum class TNetSocketSendState{ WAIT, WAIT_FOR_ACK, LOWER_ACK };

#define RECV_BUFFER_LEN 128

class udp_client_server_runtime_error : public std::runtime_error
{
public:
    udp_client_server_runtime_error(const char *w) : std::runtime_error(w) {}
};


class udp_client{
	
private:
    int                 f_socket;
    int                 f_port;
    std::string         f_addr;
    struct addrinfo *   f_addrinfo;
	
public:
	udp_client(const std::string& addr, int port);
	~udp_client();

	int get_socket() const;
	int get_port() const;
	std::string get_addr() const;

	int send(const char *msg, size_t size);
};


class udp_server{

public:
	udp_server(const std::string& addr, int port);
	~udp_server();

	int get_socket() const;
	int get_port() const;
	std::string get_addr() const;

	int recv(char *msg, size_t max_size);
	int timed_recv(char *msg, size_t max_size, int max_wait_ms);

private:
    int                 f_socket;
    int                 f_port;
    std::string         f_addr;
    struct addrinfo *   f_addrinfo;
};


/**
 * @class TNetSocket
 * @author Anderson Domingues
 * @date 11/19/18
 * @file TNetSocket.h
 * @brief This class defines a model for a "fake" network adapter 
 * that uses of UDP datagrams at one side and on-chip network's
 * flits on the other side of the UCommunication.
 */
class TNetSocket: public TimedModel{

private:
	//This module supports sending and receiving
	//packet simultaneously, so we keep the states
	//of two state machines below.
	TNetSocketSendState _send_state; 
	TNetSocketRecvState _recv_state;
	
	//Control wires interfacing with the CPU. The protocol is
	//as follows:
	//1) When flits come from the router, wait until all flits 
	//   get received and then interrupt the cpu (raise _comm_intr),
	//   and wait for _comm_ack to raise until sending again.
	//2) When _comm_start raises, copy contents from mem1 onto router's
	//   buffer, then raise _comm_status.
	UComm<int8_t>* _comm_ack;    //IN
	UComm<int8_t>* _comm_start;  //IN
	UComm<int8_t>* _comm_status; //OUT
	UComm<int8_t>* _comm_intr;   //OUT

	//Control wire for the receiving thread. This module uses 
	//an additional thread that treats UDP packets outside the simulation
	//thread. The comm below is used to communicated  between threads.
	//@TODO: replace by the non-blocking model
	int8_t _recv_val;
	UComm<int8_t>* _comm_recv;  //binds to control[3]

	//interface with memories
	UMemory* _mem1; //packets to be received
	UMemory* _mem2; //packets to be sent
    
	ofstream output_debug;
	 
	//file descriptor for the sending and receiving through sockets
	int32_t _send_socket;
	int32_t _recv_socket;

   //references to udp client and server objects
	udp_client* _udp_client;
	udp_server* _udp_server;
	
   //packet counter
	uint32_t _trafficOut;
	uint32_t _trafficIn;

   //number of flits to receive from the noc before 
   //send to the udp network (read from the second flit)
	uint32_t _flits_to_recv; 

   //memory space to store packets received from the network
	uint8_t _recv_buffer[RECV_BUFFER_LEN];
public:	
    
	//returns current output
	void LogWrite(std::string);
	
	//returns a pointer to the receiving buffer
	uint8_t* GetBuffer();
	
   //internal processes
   void udpToNocProcess();
   void nocToUdpProcess();
	
	//thread for receiving (non-block)
	static void* udpRecvThread(void*);
	udp_server* GetUdpServer();

	//other 
	unsigned long long Run();
	void Reset();

	//setters/getters for memories
	UMemory* GetMem1();
	UMemory* GetMem2();
	void SetMem1(UMemory*);
	void SetMem2(UMemory*);
	
	//setter/getters for UComms
	UComm<int8_t>* GetCommIntr();
	UComm<int8_t>* GetCommStart();
	UComm<int8_t>* GetCommAck();
	UComm<int8_t>* GetCommStatus();


	UComm<int8_t>* GetCommRecv();
	
	void SetCommIntr(UComm<int8_t>* UComm);
	void SetCommStart(UComm<int8_t>* UComm);
	void SetCommAck(UComm<int8_t>* UComm);

   //ctor./dtor.
   TNetSocket(string name);
   ~TNetSocket();
};

#endif /* __TNETSOCKET_H */
