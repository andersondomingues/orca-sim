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
#ifndef MODELS_ORCA_NETWORK_INTERFACE_INCLUDE_DMANETIF_HPP_
#define MODELS_ORCA_NETWORK_INTERFACE_INCLUDE_DMANETIF_HPP_

// usually equals to routers' buffer len
#ifndef NI_BUFFER_LEN
#define NI_BUFFER_LEN 16
#pragma message "NI buffer length undefined, defaulting to 16"
#endif

// std API
#include <iostream>
#include <string>

// simulator API
#include "TimedModel.hpp"
#include "Buffer.hpp"
#include "Memory.hpp"
#include "Signal.hpp"

using orcasim::base::TimedModel;
using orcasim::base::SimulationTime;
using orcasim::modeling::Memory;
using orcasim::modeling::Signal;
using orcasim::modeling::Buffer;

namespace orcasim::models::orca {

/**
 * @brief flit
 * 
 */
// @todo(ad): maybe make it generic<T>
typedef uint16_t FlitType;

/**
 * @brief blabla
 * 
 */
enum class DmaNetifRecvState{

    RELOAD_WAIT,       // wait for a flit to "wake up" the ni
    RELOAD_SIZE,       // wait for the size of the burst
    RELOAD_COPY,       // copy raw payload into memory
    RELOAD_FLUSH,      // ?

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
 * @class DmaNetif
 * @author Anderson Domingues
 * @date 10/03/18
 * @file TDmaNetif.h
 * @brief This class models the behaviour of a network
 * interface assuming dma tranferring
 */
class DmaNetif: public TimedModel{
 private:
    // Pointer to main memory, recv mem, and send mem
    Memory* _mem0;
    Memory* _mem1;  // recv_mem
    Memory* _mem2;  // send_mem

    // States for send and recv processes
    DmaNetifRecvState _recv_state;
    DmaNetifSendState _send_state;

    // store temporary flits
    FlitType _recv_reg;
    FlitType _send_reg;

    // CPU interface
    Signal<uint8_t>* _sig_stall;  // IN
    Signal<uint8_t>* _sig_intr;   // IN
    Signal<uint8_t>* _sig_recv_reload;  // IN
    Signal<uint8_t>* _sig_prog_send;  // IN
    Signal<uint8_t>* _sig_prog_recv;  // IN
    Signal<uint8_t>* _sig_send_status;  // OUT

    Signal<uint32_t>* _sig_recv_status;  // OUT
    Signal<uint32_t>* _sig_prog_addr;  // IN
    Signal<uint32_t>* _sig_prog_size;  // IN

    Signal<uint16_t>* _sig_prog_dest;  // IN

    // recv specific vars
    uint32_t _recv_payload_size;       // total size of the payload (flits)
    uint32_t _recv_payload_remaining;  // number of flits received or sent
    uint32_t _recv_address;            // memory position to which to write to

    // send specific vars
    uint32_t _send_payload_size;       // total size of the payload (flits)
    uint32_t _send_payload_remaining;  // number of flits to copy to the noc
    uint32_t _send_address;          // memory position to which to read from

    // NoC router interface. Both the NI and Router have buffers at the input
    Buffer<FlitType>* _ib;
    Buffer<FlitType>* _ob;

 public:
    // state getters
    DmaNetifRecvState GetRecvState();
    DmaNetifSendState GetSendState();

    // getters
    Signal<uint8_t>*  GetSignalStall();
    Signal<uint8_t>*  GetSignalIntr();
    Signal<uint8_t>*  GetSignalProgSend();
    Signal<uint8_t>*  GetSignalProgRecv();
    Signal<uint8_t>*  GetSignalRecvReload();
    Signal<uint8_t>*  GetSignalSendStatus();
    
    Signal<uint32_t>* GetSignalRecvStatus();
    Signal<uint32_t>* GetSignalProgAddr();
    Signal<uint32_t>* GetSignalProgSize();

    Signal<uint16_t>* GetSignalProgDest();

    // setters
    void SetSignalStall(Signal<uint8_t>*);
    void SetSignalIntr(Signal<uint8_t>*);
    void SetSignalSendStatus(Signal<uint8_t>*);
    void SetSignalProgSend(Signal<uint8_t>*);
    void SetSignalProgRecv(Signal<uint8_t>*);
    void SetSignalRecvReload(Signal <uint8_t>*);

    void SetSignalRecvStatus(Signal<uint32_t>*);
    void SetSignalProgAddr(Signal<uint32_t>*);
    void SetSignalProgSize(Signal<uint32_t>*);

    void SetSignalProgDest(Signal<uint16_t>*);

    // internal processes
    void sendProcess();
    void recvProcess();

    // other
    SimulationTime Run();
    void Reset();

    // memories
    void SetMem0(Memory*);
    void SetMem1(Memory*);
    void SetMem2(Memory*);

    // buffers
    void SetOutputBuffer(Buffer<FlitType>* ob);  // packets go to router
    Buffer<FlitType>* GetInputBuffer();         // packets come from router

    // ctor./dtor.
    explicit DmaNetif(std::string name);
    ~DmaNetif();
};


}  // namespace orcasim::models::orca
#endif  // MODELS_ORCA_NETWORK_INTERFACE_INCLUDE_DMANETIF_HPP_
