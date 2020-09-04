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
#ifndef PLATFORMS_SINGLE_CORE_EXT_INCLUDE_SINGLECOREEXT_HPP_
#define PLATFORMS_SINGLE_CORE_EXT_INCLUDE_SINGLECOREEXT_HPP_

#include "Simulator.hpp"

using orcasim::modeling::Simulator;

using orcasim::modeling::Memory;
using orcasim::models::hfriscv::HFRiscV;
using orcasim::models::orca::NetBridge;
using orcasim::models::orca::DmaNetif;

namespace orcasim::platforms::singlecoreext {

class SingleCoreExt : Simulator {
 private:
    // signal list
    Signal<uint8_t> *signal_stall, *signal_intr, *signal_send_status,
        *signal_send, *signal_recv;
    Signal<uint32_t> *signal_addr, *signal_size, *signal_recv_status;

    // "big modules"
    Memory *mem, *mem1, *mem2;  // main memory, plus two buffer for the ni
    HFRiscV* cpu;
    NetBridge* bridge;
    DmaNetif* netif;
 public:
    void Startup();  // model instantiation
    void Simulate();  // simulation
    void Report();   // statistics
    void Cleanup();  // delete instances
};

}  // namespace orcasim::platforms::singlecoreext
#endif  // PLATFORMS_SINGLE_CORE_EXT_INCLUDE_SINGLECOREEXT_HPP_
