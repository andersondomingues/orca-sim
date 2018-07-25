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

//States for internal state machines
enum class SendState {WAIT, LOAD, COPY_FROM_MEM, FINISH};
enum class RecvState {WAIT, COPY_TO_MEM, FINISH}; 
enum class NocState {HEADER, PAYLOAD, DATA};
enum class ArbiterState {ROUND, SEND, RECEIVE};

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
        //internal module states
		ArbiterState _arbState;
		SendState    _sendState;
        RecvState    _recvState;
        NocState     _nocState;
        
        //signals
        uint32_t _bufferr[TAM_BUFFER_DMNI], _intr_count;
        bool _is_header[TAM_BUFFER_DMNI];
        
        uint32_t _first, _last;
        bool _add_buffer;
        
        uint32_t _payload_size;
        
        uint32_t _timer;
        uint32_t _send_address, _send_address_2, _send_size, _send_size_2;
        uint32_t _recv_address, _recv_size;
        bool _prio, _operation, _read_av, _slot_available;
        bool _read_enable, _write_enable;
        
        bool _send_active_2, _receive_active_2;
        uint32_t _intr_counter_temp;
        
        uint32_t _size, _size_2;
        uint32_t _address, _address_2;
        
        //configuration interface (IN)
        bool _set_address, _set_address_2, 
              _set_size, _set_size_2, 
              _set_op, _start;
        uint32_t* _config_data;
        
        //status outputs (OUT)
        bool _intr, _send_active, _receive_active;
        
        //memory interface
        uint32_t _mem_address, _mem_data_write;
        
        uint32_t* _mem_data_read;
        uint8_t  _mem_byte_we; //should be std_logic_vector(3 downto 0);
        
        //noc interface (local port)        
        bool _tx, _credit_o; //(OUT)
        bool* _rx;
        bool* _credit_i; //(IN)
        RegFlit _data_out;  //(OUT)
        RegFlit* _data_in;    //(IN)
        
public: 
        //status
        bool* GetIntr();
        bool* GetSendActive();
        bool* GetReceiveActive();
        
        //memory
        uint32_t* GetMemAddress();
        uint32_t* GetMemDataWrite();
        uint8_t*  GetMemByteWe();
        
        //noc interface
        bool* GetTx();
        RegFlit* GetDataOut();
        bool* GetCreditO();
        
        /**
         * @brief 
         * @param set_address
         * @param set_address_2
         * @param set_size
         * @param set_size_2
         * @param set_op
         * @param start
         * @param config_data
         * @param mem_data_read
         * @param credit_i
         * @param rx
         * @param data_in
         */
        void PortMap(
        
            //configuration if
            bool* set_address,
            bool* set_address_2,
            bool* set_size,
            bool* set_size_2,
            bool* set_op,
            bool* start,
            uint32_t* config_data,
            
            //memory if
            uint32_t* mem_data_read,
            
            //noc if (local port)
            bool* credit_i,
            bool* rx,
            RegFlit* data_in
        );
        
		/** Implementation of the Process' interface
		  * @return time taken for perming next cycle */
		unsigned long long Run();
		
		/** Ctor.
		  * @param name: name for this instance of Process
		  * @param mem: memory model to attach the dmni to
		  * @param intr: interrupt address
		  * @param mmr: mmr address */
		DmniModel(string name);
	
		/** Dtor. */
		~DmniModel();
		
        //Abstraction of inernal processes
		void CycleArbiter();
		void CycleReceive();
		void CycleSend();
        
        /**
         * @brief Configure operation, affect only input of current 
         * state. We opt for placing configure here so that it could
         * be called externally. */
		void CycleConfigure();
		
		/**
		 * @brief Reset operation, affects all internal state machines. We opt for
         * placing reset here so that the model would not check for the reset flag
         * at each cycle. Therefore, the DMNI must be reseted from elsewhere in the 
         * simulation. Since we found no use for reseting it but at the beggining, 
         * we call this function during object construction. */
		void Reset();
};


#endif /* DMNI */
