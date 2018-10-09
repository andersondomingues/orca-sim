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
#ifndef __TNETIF_H
#define __TNETIF_H

//std API
#include <iostream>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>
#include <UMemory.h>
#include <UComm.h>

typedef uint16_t FlitType;

enum class NetifRecvState{ READY, DATA_IN, INTR_CPU, WAIT};
enum class NetifSendState{ READY, SETUP, DATA_OUT, WAIT};

/**
 * @class TNetif
 * @author Anderson Domingues
 * @date 10/03/18
 * @file TNetif.h
 * @brief This class models the behaviour of a network
 * interface based on the one from HermesSRC platform
 */
class TNetif: public TimedModel{

private:
    //memories for reading and writing network packets
    //by convetion, _mem0 should be used to address the
    //main memory module. Received packets must be written
    //to _mem1, while packets to be sent must be written
    //to _mem2.
    UMemory* _mem1; //packets to be received
    UMemory* _mem2; //packets to be sent
    
    //this module can send and receive packets simultaneously.
    //the following states reffer to the sender and receiver 
    //processes.
    NetifRecvState _recv_state; //state of receiver module
    NetifSendState _send_state; //state of sender module

    //recv proc vars
    uint32_t _flits_to_send; //to router
    uint32_t _next_send_addr; 
	
	uint32_t _flits_to_recv; //to memory
	uint32_t _next_recv_addr; 
    

    //communication with CPU while receiving
    UComm<bool>* _comm_intr; //up when packet arrive, down when ack
    UComm<bool>* _comm_ack;  //ack when cpu finishes copying to main memory

    //communication with CPU while sending
    UComm<bool>* _comm_start;  //cpu set up to send, down by netif when finished
    UComm<bool>* _comm_status;  //cpu set up to send, down by netif when finished
    
    //network router interface
    UBuffer<FlitType>* _ib;
    UBuffer<FlitType>* _ob;
    
public:	
    
    void SetCommAck(UComm<bool>* comm);
    void SetCommStart(UComm<bool>* comm);
    void SetCommStatus(UComm<bool>* comm);
    void SetCommIntr(UComm<bool>* comm);
    
    //internal processes
    void sendProcess();
    void recvProcess();

    //other 
    unsigned long long Run();
    void Reset();

	//memories
	void SetMem1(UMemory*);
	void SetMem2(UMemory*);

	//buffers
	void SetOutputBuffer(UBuffer<FlitType>* ob); //packets go to the router
	UBuffer<FlitType>* GetInputBuffer();         //packets come from the router

    //ctor./dtor.
    TNetif(string name);
    ~TNetif();
};


#endif /* TNETIF_H */
