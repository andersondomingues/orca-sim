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
#ifndef __TDMANETIF_H
#define __TDMANETIF_H

//usually equals to routers' buffer len
#define NI_BUFFER_LEN 16

//std API
#include <iostream>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>
#include <UMemory.h>
#include <USignal.h>

typedef uint16_t FlitType;

enum class DnRecvState{ WAIT, READ_LEN, REQ_ADDR, WAIT_ADDR, WAIT_FLITS, MEM_WRITE, RELEASE};
enum class DnSendState{ READY, LENGTH, DATA_OUT};

/**
 * @class TDmaNetif
 * @author Anderson Domingues
 * @date 10/03/18
 * @file TDmaNetif.h
 * @brief This class models the behaviour of a network
 * interface assuming dma tranferring
 */
class TDmaNetif: public TimedModel{

private:

	//Pointer to main memory. Memory interface is abstracted (op, addr, data)
	UMemory _mem0;

	/** state of recv process **/
    DnRecvState _recv_state;
    
    /** number of flits to expect for current burst **/
	uint32_t _recv_flits_to_go; 
	
	/** address to which a flit will be writen in this cycle **/
	uint32_t _recv_current_addr;
	
	/** additional spaces to store addr and size flits **/
	FlitType _recv_addr_flit;
	FlitType _recv_size_flit;

	/** handshake for address request **/
	USignal<int8_t>* _sig_intr;      //<-- request cpu interruption signal (same for both processes)
	
	/** recving program
	USignal<int8_t>* _sig_addr_ack;  //<-- cpu ack with new address
	USignal<int8_t>* _sig_addr_data; //<-- place where cpu store the addr



	//=================== REST OF STUFF
    DmaNetifSendState _send_state; //state of sender module



    //recv proc vars
    uint32_t _flits_to_send; //to router
    uint32_t _next_send_addr; 
	

    
    //signalunication with CPU while receiving
    USignal<int8_t>* _signal_intr; //up when packet arrive, down when ack
    USignal<int8_t>* _signal_ack;  //ack when cpu finishes copying to main memory

    //signalunication with CPU while sending
    USignal<int8_t>* _signal_start;  //cpu set up to send, down by netif when finished
	USignal<int8_t>* _signal_status; //status of sending process (required by cpu)
    
    //NOC router interface. Both the NI and Router has buffers at the input (not output).
    UBuffer<FlitType>* _ib;
    UBuffer<FlitType>* _ob;
public:	
    
    //getters
    DmaNetifRecvState GetRecvState();
	DmaNetifSendState GetSendState();
    
    //setters
    void SetSignalAck(USignal<int8_t>* signal);
	void SetSignalIntr(USignal<int8_t>* signal);
	void SetSignalStart(USignal<int8_t>* signal);
	void SetSignalStatus(USignal<int8_t>* signal);
	
	USignal<int8_t>* GetSignalAck();
	USignal<int8_t>* GetSignalIntr();
	USignal<int8_t>* GetSignalStart();
	USignal<int8_t>* GetSignalStatus();
    
    //internal processes
    void sendProcess();
    void recvProcess();

    //other 
    SimulationTime Run();
    void Reset();

	//memories
	void SetMem1(UMemory*);
	void SetMem2(UMemory*);

	//buffers
	void SetOutputBuffer(UBuffer<FlitType>* ob); //packets go to the router
	UBuffer<FlitType>* GetInputBuffer();         //packets come from the router

    //ctor./dtor.
    TDmaNetif(string name);
    ~TDmaNetif();
};


#endif /* __TDMANETIF_H */
