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
#ifndef __TDMNI_H
#define __TDMNI_H

//std API
#include <iostream>

//simulator API
#include <TimedModel.h>

//model API
#include <UMemory.h>
#include <UBuffer.h>

#define DMNI_TIMER 10  /*std_logic_vector(4 downto 0):="10000"*/
#define WORD_SIZE  4   /*std_logic_vector(4 downto 0):="00100"*/

/*
 * The RegFlit type affects both routers from Hermes and 
 * the DMNI modules. Before changing it, make sure that 
 * the change will correcly propagate to all involved 
 * modules*/
typedef uint16_t FlitType;

//memory-mapped "registers"

#define DMNI_SIZE 	        0xf0000100
#define DMNI_OP   	        0xf0000104
#define DMNI_ADDRESS        0xf0000108
#define DMNI_START 	        0xf000010C

#define DMNI_SEND_ACTIVE    0xf0000110
#define DMNI_RECEIVE_ACTIVE 0xf0000114

#define DMNI_WRITE 2
#define DMNI_READ  1
#define DMNI_NONE  0

//States for internal state machines
enum class SendState {WAIT, FINISH, COPY_FROM_MEM};
enum class RecvState {WAIT, FINISH, COPY_TO_MEM};
enum class ArbiterState {SEND, RECV, ROUND}; //ok
/**
 * @class DmniModel
 * @author Anderson Domingues
 * @date 07/22/18
 * @file DmniModel.h
 * @brief This class implements the behaviour of the DMNI module
 * from HeMPS project (see URSA documentation regarding HeMPS). The
 * DMNI module has both roles of NI (network interface) and DMA (
 * direct memory access) modules. */
class TDmni : public TimedModel {

private:
        //dmin->proc interface
        bool* _intr;
        
        //proc->dmni interface
        int8_t _mma_addr; //addr to start copying from
        int8_t _mma_len;  //length of data
        int8_t _mma_op;
        
        //memory interface
        UMemory* _mem;
        
        //noc interface (local port)        
        UBuffer<FlitType>* _ib;
        UBuffer<FlitType>* _ob; 
        
        bool _start;
        
        //private aux
        RecvState _recv_state;
        uint32_t _recv_addr;
        uint32_t _recv_len;
        
        SendState _send_state;
        uint32_t _send_addr;
        uint32_t _send_len;
        
        ArbiterState _arb_state;
        
        //--
        bool _read_enable, _write_enable, _prio;
        bool _send_active, _recv_active;
        uint32_t _timer;
        
public:       
        //Noc IO
        UBuffer<FlitType>* GetOutputBuffer();
        UBuffer<FlitType>* GetInputBuffer();
        
		void SetOutputBuffer(UBuffer<FlitType>*);
		void SetIntr(bool* b);
		
		uint8_t GetSendActive();
		uint8_t GetReceiveActive();
		
		/**
		 * @brief Sends data from the memory to the 
		 * network router. Data is identified by a
		 * starting address and size.
		 * @param addr Address in which data begins.
		 * @param size Total length o data (size of FlitType) */
		void CopyFrom(uint32_t addr, uint32_t size);
		
		/**
		 * @brief Receives data from the network router and 
		 * stores it into the memory. 
		 * @param addr Address to start the writing.
		 * @param size Size of data to be written.*/
		void CopyTo(uint32_t addr, uint32_t size);
		
		
        //memory
        void SetMemoryModel(UMemory* mem);
        UMemory* GetMemoryModel();
        
		
        /** Implementation of the Process' interface
		  * @return time taken for perming next cycle */
		unsigned long long Run();
        void CycleSend();
        void CycleReceive();
        void CycleArbiter();
		
		/** Ctor.
		  * @param name: name for this instance of Process
		  * @param mem: memory model to attach the dmni to
		  * @param intr: interrupt address
		  * @param mmr: mmr address */
		TDmni(string name);
	
		/** Dtor. */
		~TDmni();

		/**
		 * @brief Reset operation, affects all internal state machines. We opt for
         * placing reset here so that the model would not check for the reset flag
         * at each cycle. Therefore, the DMNI must be reseted from elsewhere in the 
         * simulation. Since we found no use for reseting it but at the beggining, 
         * we call this function during object construction. */
		void Reset();
};

#endif /* TDMNI_H */
