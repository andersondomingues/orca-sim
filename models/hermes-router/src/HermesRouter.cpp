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
#include <cstdlib>
#include <sstream>
#include <iomanip>

#include "HermesRouter.hpp"

using orcasim::models::hermes::FlitType;
using orcasim::models::hermes::HermesRouter;
using orcasim::base::SimulationTime;
using orcasim::modeling::Buffer;

/**
 * @brief Get the name of the port of id equals to <port>
 * @param port the id of the port
 * @return and instance of std::string containing port's name */
std::string HermesRouter::GetPortName(int port) {
    switch (port) {
        case SOUTH: return "SOUTH";
        case NORTH: return "NORTH";
        case WEST:  return "WEST";
        case EAST:  return "EAST";
        default: return "LOCAL";
    }
    return "???";
}

/**
 * @brief Ctor.
 * @param name Name of the instance (proccess impl)
 * @param x_pos X coordinate (first part of router addr)
 * @param y_pos Y coordinate (second part of router addr) */
HermesRouter::HermesRouter(std::string name, uint32_t x_pos, uint32_t y_pos):
    TimedModel(name) {
    _x = x_pos;
    _y = y_pos;

    // for all ports, create a new input buffer; Note that data is bufferred by
    // input buffers, since output buffers come from somewhere else;
    for (int i = 0; i < 5; i++) {
        std::string bname = GetName() +  ".IN-" + this->GetPortName(i);
        _ob[i] = nullptr;
        _ib[i] = new Buffer<FlitType>(bname, BUFFER_CAPACITY);
    }

    #ifdef ROUTER_ENABLE_COUNTERS
    // if counters are enabled, initiate the respective signals
    // please note that these signals are not mapped to anywhere yet
    _counter_active = new Signal<uint32_t>(GetName() + ".counters.active");
    #endif

    this->Reset();
}

/**
 * @brief Free allocated memory if any
 */
HermesRouter::~HermesRouter() {
    #ifdef ROUTER_ENABLE_COUNTERS
    delete _counter_active;
    #endif

    for (int i = 0; i < 5; i++)
        delete(_ib[i]);
}

void HermesRouter::Reset() {
    _round_robin = LOCAL;  // starts checking on local port

    for (int i = 0; i < 5; i++) {
        _flits_to_send[i] = 0;
        _switch_control[i] = -1;
    }

    // TODO(ad): reset buffers
}

uint32_t HermesRouter::GetRR() {
    return _round_robin;
}

/**
 * @brief Implementation of the Run method from
 * the Proccess abstract class.
 * @return The next time to schedule the event.*/
SimulationTime HermesRouter::Run() {
    // CROSSBAR CONTROL: connect priority port to destination if it has any
    // packet to send but is waiting for the destination to free
    if (_ib[_round_robin]->size() > 0 && _switch_control[_round_robin] == -1) {
        // find the destination using the address in the first flit
        uint8_t target_port = this->GetRouteXY(_ib[_round_robin]->top());

        // check whether the destination port is bound to some other source port
        bool bound = false;

        for (int i = 0; i < 5; i++) {
            if (_switch_control[i] == target_port) {
                bound = true;
                break;
            }
        }

        // if the port is not bind, binds it to the source
        if (!bound) {
            // set crossbar connection
            _switch_control[_round_robin] = target_port;
            // -2 means "the size flit has not arrived yet"
            _flits_to_send[_round_robin] = -2;
        }
      }

    // drive flits into destination ports
    for (int i = 0; i < 5; i++) {
        // check whether the switch control is closed for some port
        if (_switch_control[i] != -1) {
            // prevent routing to a non-existing router
            #ifdef ROUTER_PORT_CONNECTED_CHECKING
            if (_ob[_switch_control[i]] == nullptr) {
                stringstream ss;
                ss << this->GetName() << ": unable to route to unknown port"
                    << std::endl;
                std::runtime_error(ss.str());
            }

            if (_ib[i] == nullptr) {
                stringstream ss;
                ss << this->GetName() << ": unable to route from unknown port"
                    << std::endl;
                std::runtime_error(ss.str());
            }
            #endif
            // check whether the output is able to receive new flits.
            // buffer must have some room
            if (!_ob[_switch_control[i]]->full() && _ib[i]->size() > 0) {
                // if -2, we send the address flit
                if (_flits_to_send[i] == -2) {
                    // push one flit to destination port
                    _ob[_switch_control[i]]->push(_ib[i]->top());
                    _flits_to_send[i] = -1;
                    _ib[i]->pop();  // remove flit from source port

                // if -1, we set the size flit and send it
                } else if (_flits_to_send[i] == -1) {
                    _flits_to_send[i] = _ib[i]->top();
                     // push one flit to destination port
                    _ob[_switch_control[i]]->push(_ib[i]->top());
                    _ib[i]->pop();  // remove flit from source port

                } else {
                    _flits_to_send[i] -= 1;
                     // push one flit to destination port
                    _ob[_switch_control[i]]->push(_ib[i]->top());
                    _ib[i]->pop();  // remove flit from source port
                }

                #ifdef ROUTER_ENABLE_COUNTERS
                _is_active = true;
                #endif

                // free port
                if (_flits_to_send[i] == 0)
                    _switch_control[i] = -1;
            }
        }
    }

    // round robin update
    _round_robin = (_round_robin + 1) % 5;

    #ifdef ROUTER_ENABLE_COUNTERS
    if (is_active) {
        _counter_active->Inc(1);
        _is_active = false;
    }
    #endif
    return 1;
}

#ifdef ROUTER_ENABLE_COUNTERS
Signal<uint32_t>* TRouter::GetSignalCounterActive() {
    return this->_counter_active;
}

#endif

/**
 * @brief Calculate the port to route a given flit
 * @param flit to be routed
 * @return the port to where te packet must go*/
uint32_t HermesRouter::GetRouteXY(FlitType flit) {
    FlitType tx = (flit & 0x00F0) >> 4;
    FlitType ty = flit & 0x000F;

    // if X=0, then route "vertically" (Y)
    if (_x == tx) {
        return (_y == ty)
            ? LOCAL
            : (_y > ty)
                ? SOUTH
                : NORTH;
    // route X
    } else {
        return (_x > tx)
            ? WEST
            : EAST;
    }
}

/**
 * @brief Get a pointer to one of the output buffers.
 * @param r The port from which get the pointer.
 * @return A pointer to the buffer.*/
Buffer<FlitType>* HermesRouter::GetOutputBuffer(uint32_t r) {
    return _ob[r];
}

Buffer<FlitType>* HermesRouter::GetInputBuffer(uint32_t r) {
    return _ib[r];
}

void HermesRouter::SetOutputBuffer(Buffer<FlitType>* b, uint32_t port) {
    _ob[port] = b;
}

std::string HermesRouter::ToString() {
    std::stringstream ss;
    ss << this->GetName() + ": ";

    for (int i = 0; i < 5; i++) {
        if (_ob[i] != nullptr)
            ss << "{" << _ob[i]->GetName() << "} ";
    }

    return ss.str();
}
