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
#ifndef MODELS_INCLUDE_TROUTER_HPP_
#define MODELS_INCLUDE_TROUTER_HPP_

// std API
#include <iostream>
#include <string>

// simulator API
#include "TimedModel.hpp"
#include "UBuffer.hpp"
#include "USignal.hpp"

typedef uint16_t FlitType;

enum class RouterState{
    ROUNDROBIN, FORWARD1, PKTLEN, BURST
};

// routing ports
#define NORTH 0
#define WEST  1
#define SOUTH 2
#define EAST  3

#define LOCAL 4

class TRouter: public TimedModel{
 private:
    #ifdef ROUTER_ENABLE_COUNTERS
    USignal<uint32_t>* _counter_active;
    #endif

    #ifdef ROUTER_ENABLE_COUNTERS
    bool is_active = false;
    #endif

    // stores info about actively sending ports. For intance, position zero
    // representing the status of LOCAL port. The value in the position indicate
    // to which port the LOCAL port is sending to. Inactive ports have -1
    // written to their position.
    int16_t _switch_control[5];

    // stores how many flits the must be forwarded to the destination port
    int16_t _flits_to_send[5];

    // stores which port has the priority acconding to the round robin policy
    uint32_t _round_robin;

    // address of the router
    uint32_t _x, _y;

    // output buffers
    UBuffer<FlitType>* _ob[5];

    // input buffers
    UBuffer<FlitType>* _ib[5];

 public:
    #ifdef ROUTER_ENABLE_COUNTERS
    USignal<uint32_t>* GetSignalCounterActive();
    #endif

    uint32_t GetRR();

    UBuffer<FlitType>* GetOutputBuffer(uint32_t p);
    UBuffer<FlitType>* GetInputBuffer(uint32_t p);

    void SetOutputBuffer(UBuffer<FlitType>* b, uint32_t port);

    /** Implementation of the Process' interface
      * @return time taken for perming next cycle */
    SimulationTime Run();

    /** return this **/
    uint32_t GetRouteXY(FlitType flit);

    /** Ctor. **/
    TRouter(std::string name, uint32_t x_pos, uint32_t y_pos);

    /** Dtor. **/
    ~TRouter();

    /**
     * @brief Get the name of the port of id equals to <port>
     * @param port the id of the port
     * @return and instance of std::string containing port's name */
    std::string GetPortName(int port);

    void Reset();
    std::string ToString();
};

#endif  // MODELS_INCLUDE_TROUTER_HPP_
