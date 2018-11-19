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

//simulator API
#include <TimedModel.h>

//dependency models
#include <UBuffer.h>
#include <UMemory.h>
#include <UComm.h>
#include <TNetif.h>

typedef uint16_t FlitType;

enum class NetSocketSendState{ READY, LENGTH, DATA_IN, INTR_AND_WAIT};
enum class NetSocketRecvState{ READY, LENGTH, DATA_OUT};

/**
 * @class TNetSocket
 * @author Anderson Domingues
 * @date 11/19/18
 * @file TNetSocket.h
 * @brief This class defines a model for a "fake" network adapter 
 * that uses of UDP datagrams at one side and on-chip network's
 * flits on the other side of the communication.
 */
class TNetSocket: public TimedModel{

private:
   
	//interface with memories from netif
    UMemory* _mem1; //packets to be received
    UMemory* _mem2; //packets to be sent
    
    //as netif, this module supports sending and receiving
	//packet simultaneously.
    NetSocketRecvState _recv_state; //state of receiver module
    NetSocketSendState _send_state; //state of sender module
	
	//file descriptor for the sending and receiving sockets
	int32_t _send_socket;
	int32_t _recv_socket;
	
	//adresses for of the running host
	//struct sockaddr_in _sock_addr; 
	
public:	
    
    //internal processes
    void sendProcess();
    void recvProcess();

    //other 
    unsigned long long Run();
    void Reset();

	//memories
	void SetNetif(TNetif*);
	TNetif* GetNetif();

    //ctor./dtor.
    TNetSocket(string name);
    ~TNetSocket();
};


#endif /* __TNETSOCKET_H */
