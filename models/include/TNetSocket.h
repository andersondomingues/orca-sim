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

	ofstream output_debug;
   
	//interface with memories from netif
    UMemory* _mem1; //packets to be received
    UMemory* _mem2; //packets to be sent
    
	//as netif, this module supports sending and receiving
	//packet simultaneously.
	TNetSocketSendState _send_state; 
	TNetSocketRecvState _recv_state;
	
	//control wires (netsocket <-> netif)
	UComm<int8_t>* _comm_ack;
	UComm<int8_t>* _comm_intr;
	UComm<int8_t>* _comm_start;
	
	UComm<int8_t>* _comm_recv;
	
	//adresses for of the running host
	//std::thread _recv_thread;
	
	//file descriptor for the sending and receiving sockets
	int32_t _send_socket;
	int32_t _recv_socket;
	
	udp_client* _udp_client;
	udp_server* _udp_server;
	
	uint32_t _trafficOut;
	uint32_t _trafficIn;
	uint32_t _flits_to_recv; 
	
	uint8_t* _recv_buffer;
	
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
	
	UComm<int8_t>* GetCommRecv();
	
	void SetCommIntr(UComm<int8_t>* UComm);
	void SetCommStart(UComm<int8_t>* UComm);
	void SetCommAck(UComm<int8_t>* UComm);

    //ctor./dtor.
    TNetSocket(string name);
    ~TNetSocket();
};


#endif /* __TNETSOCKET_H */
