/******************************************************************************
 * This file is part of project ORCA. More information on the project
 * can be found at the following repositories at GitHub's website.
 *
 * http://https://github.com/andersondomingues/orca-sim
 * http://https://github.com/andersondomingues/orca-software
 * http://https://github.com/andersondomingues/orca-mpsoc
 * http://https://github.com/andersondomingues/orca-tools
 *
 * Copyright (C) 2018-2020 Anderson Domingues, <ti.andersondomingues@gmail.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. 
******************************************************************************/
#ifndef MODELS_INCLUDE_TDMANETIF_HPP_
#define MODELS_INCLUDE_TDMANETIF_HPP_

// usually equals to routers' buffer len
#define NI_BUFFER_LEN 16

// std API
#include <iostream>
#include <string>

// simulator API
#include "TimedModel.hpp"
#include "UBuffer.hpp"
#include "UMemory.hpp"
#include "USignal.hpp"

/**
 * @brief flit
 * 
 */
typedef uint16_t FlitType;

/**
 * @brief blabla
 * 
 */
enum class DmaNetifRecvState{
    WAIT_ADDR_FLIT,     // wait some flit to arrive at the local port
    WAIT_SIZE_FLIT,     // read size flit to determine how many will come next
    WAIT_PAYLOAD,       // wait for remaining flits to arrive, and interrupt
    WAIT_CONFIG_STALL,  // wait for the cpu to configure the dma
    COPY_RELEASE,       // stalls cpu, copy data, and release
    FLUSH               // waits for the CPU to lower the recv signal
};

/**
 * @brief blabla
 * 
 */
enum class DmaNetifSendState{
    WAIT_CONFIG_STALL,  // wait cpt to configure and raise _sig_send, stall
    COPY_AND_RELEASE,  // copy content from memory, release cpu
    SEND_DATA_TO_NOC,  // write data to the network, raise _send_status
    FLUSH              // wait for the cpu to lower the send signal (ack)
};

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
    // Pointer to main memory, recv mem, and send mem
    UMemory* _mem0;
    UMemory* _mem1;  // recv_mem
    UMemory* _mem2;  // send_mem

    // States for send and recv processes
    DmaNetifRecvState _recv_state;
    DmaNetifSendState _send_state;

    // store temporary flits
    FlitType _recv_reg;
    FlitType _send_reg;

    // OUT: stalls cpu while copying from/to main memory
    USignal<uint8_t>* _sig_stall;
    // OUT: request cpu interruption signal (same for both processes)
    USignal<uint8_t>* _sig_intr;

    // OUT: 0x0 when in ready state
    USignal<uint8_t>* _sig_send_status;
    // OUT: 0x0 when in ready state, updated but unused
    USignal<uint32_t>* _sig_recv_status;

    USignal<uint8_t>*  _sig_prog_send;   // IN
    USignal<uint8_t>*  _sig_prog_recv;   // IN

    USignal<uint32_t>* _sig_prog_addr;   // IN
    USignal<uint32_t>* _sig_prog_size;   // IN

    // recv specific vars
    uint32_t _recv_payload_size;       // total size of the payload (flits)
    uint32_t _recv_payload_remaining;  // number of flits received or sent
    uint32_t _recv_address;            // memory position to which to write to

    // send specific vars
    uint32_t _send_payload_size;       // total size of the payload (flits)
    uint32_t _send_payload_remaining;  // number of flits to copy to the noc
    uint32_t _send_address;          // memory position to which to read from

    // NOC router interface. Both the NI and Router has buffers at the input
    UBuffer<FlitType>* _ib;
    UBuffer<FlitType>* _ob;

 public:
    // getters
    DmaNetifRecvState GetRecvState();
    DmaNetifSendState GetSendState();

    // getters
    USignal<uint8_t>*  GetSignalStall();
    USignal<uint8_t>*  GetSignalIntr();

    USignal<uint8_t>*  GetSignalSendStatus();
    USignal<uint32_t>*  GetSignalRecvStatus();

    USignal<uint8_t>*  GetSignalProgSend();
    USignal<uint8_t>*  GetSignalProgRecv();

    USignal<uint32_t>* GetSignalProgAddr();
    USignal<uint32_t>* GetSignalProgSize();

    // setters
    void SetSignalStall(USignal<uint8_t>*);
    void SetSignalIntr(USignal<uint8_t>*);

    void SetSignalSendStatus(USignal<uint8_t>*);
    void SetSignalRecvStatus(USignal<uint32_t>*);

    void SetSignalProgSend(USignal<uint8_t>*);
    void SetSignalProgRecv(USignal<uint8_t>*);

    void SetSignalProgAddr(USignal<uint32_t>*);
    void SetSignalProgSize(USignal<uint32_t>*);

    // internal processes
    void sendProcess();
    void recvProcess();

    // other
    SimulationTime Run();

    void Reset();

    // memories
    void SetMem0(UMemory*);
    void SetMem1(UMemory*);
    void SetMem2(UMemory*);

    // buffers
    void SetOutputBuffer(UBuffer<FlitType>* ob);  // packets go to router
    UBuffer<FlitType>* GetInputBuffer();         // packets come from router

    // ctor./dtor.
    explicit TDmaNetif(std::string name);
    ~TDmaNetif();
};


#endif  // MODELS_INCLUDE_TDMANETIF_HPP_
