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
#ifndef MODELS_ORCA_VIRTUAL_ETHERNET_INCLUDE_NETBRIDGE_HPP_
#define MODELS_ORCA_VIRTUAL_ETHERNET_INCLUDE_NETBRIDGE_HPP_

// std API
#include <pthread.h>
#include <iostream>
#include <string>

// ursa API
#include "TimedModel.hpp"
#include "Buffer.hpp"
#include "Memory.hpp"
#include "Signal.hpp"

#ifndef NETSOCKET_CLIENT_ADDRESS
#define NETSOCKET_CLIENT_ADDRESS "127.0.0.1"
#pragma message "ip address for socket client undefined, default is localhost"
#endif

#ifndef NETSOCKET_CLIENT_PORT
#define NETSOCKET_CLIENT_PORT 5000
#pragma message "port number for socket client undefined, default is 5000"
#endif

#ifndef NETSOCKET_SERVER_ADDRESS
#define NETSOCKET_SERVER_ADDRESS "127.0.0.1"
#pragma message "ip address for socket server undefined, default is localhost"
#endif

#ifndef NETSOCKET_SERVER_PORT
#define NETSOCKET_SERVER_PORT 5001
#pragma message "port number for socket server undefined, default is 5001"
#endif

using orcasim::base::TimedModel;
using orcasim::base::SimulationTime;
using orcasim::modeling::Buffer;
using orcasim::modeling::Signal;


class udp_client_server_runtime_error : public std::runtime_error{
 public:
    explicit udp_client_server_runtime_error(const char* w)
        : std::runtime_error(w) {}
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
//--------------------------------------------

typedef uint16_t FlitType;

enum class TNetBridgeRecvState{ READY, READ_LEN, RECV_PAYLOAD};

enum class TNetBridgeSendState{ READY, SEND_LEN, SEND_PAYLOAD};

#define RECV_BUFFER_LEN 128
#define SEND_BUFFER_LEN 128

#define HWBUFFER_LEN 16

/**
 * @class TNetBridge
 * @author Anderson Domingues
 * @date 11/19/18
 * @file TNetBridge.h
 * @brief This class defines a model for a "fake" network adapter 
 * that uses of UDP datagrams at one side and on-chip network's
 * flits on the other side of the USignalunication.
 */
class TNetBridge: public TimedModel{
 private:
    // This module supports sending and receiving
    // packet simultaneously, so we keep the states
    // of two state machines below.
    TNetBridgeSendState _send_state;
    TNetBridgeRecvState _recv_state;

    // Control wire for the receiving thread. This module uses
    // an additional thread that treats UDP packets outside the simulation
    // thread. The signal below is used to signalunicated  between threads.
    // TODO(ad): replace by the non-blocking model
    int8_t _recv_val;
    Signal<int8_t>* _signal_recv;

    std::ofstream output_debug;
    std::ofstream output_uart;

    pthread_t _t;

    // file descriptor for the sending and receiving through sockets
    int32_t _send_socket;
    int32_t _recv_socket;

    // references to udp client and server objects
    udp_client* _udp_client;
    udp_server* _udp_server;

    #ifdef NETBRIDGE_ENABLE_LOG_INPUT
    uint32_t _trafficIn;
    #endif

    #ifdef NETBRIDGE_ENABLE_LOG_OUTPUT
    uint32_t _trafficOut;
    #endif

    // number of flits to receive from the noc before
    // send to the udp network (read from the second flit)
    uint32_t _flits_to_recv;
    uint32_t _flits_to_recv_count;
    uint32_t _flits_to_send;
    uint32_t _flits_to_send_count;

    // auxiliary regs
    FlitType _out_reg;

    // memory space to store packets received from the
    // noc and from the external network
    uint8_t _recv_buffer[RECV_BUFFER_LEN];
    uint8_t _send_buffer[SEND_BUFFER_LEN];

    // in and out buffer (noc side)
    Buffer<FlitType>* _ib;
    Buffer<FlitType>* _ob;

    // flag to gracefully terminate the subthread
    volatile int udpRecvThread_terminate;

 public:
    // returns current output
    void LogWrite(std::string);

    // returns a pointer to the receiving buffer
    uint8_t* GetBuffer();

    // internal processes
    void udpToNocProcess();
    void nocToUdpProcess();

    // thread for receiving (non-block)
    static void* udpRecvThread(void*);

    udp_server* GetUdpServer();

    // other
    SimulationTime Run();
    void Reset();

    // ctor./dtor.
    explicit TNetBridge(std::string name);
    ~TNetBridge();

    Signal<int8_t>* GetSignalRecv();

    Buffer<FlitType>* GetInputBuffer();
    void SetOutputBuffer(Buffer<FlitType>*);
};

#endif  // MODELS_ORCA_VIRTUAL_ETHERNET_INCLUDE_NETBRIDGE_HPP_
