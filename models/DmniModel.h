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
#ifndef __DMNI_H
#define __DMNI_H

//std API
#include <iostream>

//simulator API
#include <Process.h>
#include <Buffer.h>

//model API
#include <MemoryModel.h>

#define TAM_BUFFER_DMNI 32 
#define DMNI_TIMER 32  /*std_logic_vector(4 downto 0):="10000"*/
#define WORD_SIZE  4   /*std_logic_vector(4 downto 0):="00100"*/

/*
 * The RegFlit type affects both routers from Hermes and 
 * the DMNI modules. Before changing it, make sure that 
 * the change will correcly propagate to all involved 
 * modules*/
typedef uint32_t RegFlit;

#define OP_SEND 4
#define OP_RECV 8
#define OP_NONE 0

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
class DmniModel : public Process {

private:
        //dmin->proc interface
        bool* _intr;
        
        //proc->dmni interface
        uint32_t* _mma_addr; //addr to start copying from
        uint32_t* _mma_len;  //length of data
        uint32_t* _mma_op;
        
        //memory interface
        MemoryModel* _mem;
        
        //noc interface (local port)        
        Buffer* _ib;
        Buffer* _ob; 
        
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
        //processor interface
        bool* GetIntr();
        void SetIntr(bool*);
        
        //mma 
        void SetAddress(uint32_t* addr);
        uint32_t* GetAddress();
        void SetLength(uint32_t* len);
        uint32_t* GetLength();
        void SetOperation(uint32_t* op);
        uint32_t* GetOperation();
        
        //Noc IO
        Buffer* GetOutputBuffer();
        Buffer* GetInputBuffer();
        void SetInputBuffer(Buffer*);
        
        //memory
        void SetMemoryModel(MemoryModel* mem);
        MemoryModel* GetMemoryModel();
        
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
		DmniModel(string name);
	
		/** Dtor. */
		~DmniModel();

		/**
		 * @brief Reset operation, affects all internal state machines. We opt for
         * placing reset here so that the model would not check for the reset flag
         * at each cycle. Therefore, the DMNI must be reseted from elsewhere in the 
         * simulation. Since we found no use for reseting it but at the beggining, 
         * we call this function during object construction. */
		void Reset();
};

#endif /* DMNI */
