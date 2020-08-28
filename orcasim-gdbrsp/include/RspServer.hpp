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
#ifndef ORCASIM_GDBRSP_INCLUDE_RSPSERVER_HPP_
#define ORCASIM_GDBRSP_INCLUDE_RSPSERVER_HPP_

// base API includes
#include <iostream>
#include <list>
#include <string>

// in-package includes
#include "UdpAsyncServer.hpp"

// off-package includes
#include "ProcessorState.hpp"
#include "MemoryType.hpp"
#include "UMemory.hpp"

#define RSP_BUFFER_SIZE 5000
#define RSP_EMPTY_RESPONSE ""

using orcasim::modeling::UMemory;
using orcasim::modeling::ProcessorState;

namespace orcasim::gdbrsp {

template <typename T>
class RspServer{
 private:
    // upd server instance, see UdpServe.h
    UdpAsyncServer* _server;

    // ip address and udp port for THIS processor
    std::string _ipaddr;
    int _udpport;

    // state of the target processor and memory reference
    ProcessorState<T>* _state;
    UMemory* _memory;

    // output and input buffers
    char _output_buffer[RSP_BUFFER_SIZE];
    char _input_buffer[RSP_BUFFER_SIZE];

    // list of breakpoints
    std::list<T>* _bp_list;

 public:
    /**
     * Ctor.
     */
    RspServer(ProcessorState<T>* state, UMemory* mem, std::string ipaddr,
        uint32_t udpport);

    /**
     * Dtor.
     */
    ~RspServer();

    /**
     * Query for a message. 
     */

    int Ack();
    int Nack();

    int Receive();
    int UpdateCpuState();

    uint8_t Checksum(char* buffer, int size);
    int Respond(std::string data);

    /**
     * Message handlers
     */
    int Handle_v(char*);
    int Handle_X(char*);
    int Handle_q(char*);
    int Handle_Q(char*);
    int Handle_g(char*);
    int Handle_Question(char*);
    int Handle_c(char*);
    int Handle_C(char*);
    int Handle_s(char*);
    int Handle_S(char*);
    int Handle_H(char*);
    int Handle_k(char*);
    int Handle_m(char*);
    int Handle_M(char*);
    int Handle_p(char*);
    int Handle_P(char*);
    int Handle_Z(char*);
    int Handle_z(char*);
};

// template instantiation
template class RspServer<uint8_t>;
template class RspServer<uint16_t>;
template class RspServer<uint32_t>;
template class RspServer<uint64_t>;

template class RspServer<int8_t>;
template class RspServer<int16_t>;
template class RspServer<int32_t>;
template class RspServer<int64_t>;

}  // namespace orcasim::gdbrsp
#endif  // ORCASIM_GDBRSP_INCLUDE_RSPSERVER_HPP_

