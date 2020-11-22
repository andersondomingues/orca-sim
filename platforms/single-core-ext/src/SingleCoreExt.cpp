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
// signal manip
#include <signal.h>

#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>

// models
#include "Memory.hpp"
#include "HFRiscV.hpp"
#include "NetBridge.hpp"
#include "DmaNetif.hpp"

// orca-specific hardware
#include "MemoryMap.h"
#include "SingleCoreExt.hpp"

// simulation artifacts
#include "Event.hpp"
#include "Simulator.hpp"

using orcasim::platforms::singlecoreext::SingleCoreExt;
using orcasim::modeling::Simulator;
using orcasim::modeling::SimulatorInterruptionStatus;

SingleCoreExt::SingleCoreExt(int argc, char** argv): Simulator(argc, argv) {
    if(argc != 2){
        std::cout << "Usage: " << std::endl;
        std::cout << "\t" << argv[0] << " <software image>" << std::endl;
    }
}

/**
 * This routine regards the instantiation of hardware for the simulation. In 
 * this method we set all the private fields which corresponds to hardware 
 * modules and signals.
 */
void SingleCoreExt::Startup() {
    // instantiate signals (note that only signals that could be accessed
    // through memory must have associated address)
    signal_stall =  new Signal<uint8_t>("sig.stall");
    signal_intr  = new Signal<uint8_t>("sig.intr");
    signal_addr  = new Signal<uint32_t>(SIGNAL_PROG_ADDR, "sig.prog-addr");
    signal_size  = new Signal<uint32_t>(SIGNAL_PROG_SIZE, "sig.prog-size");
    signal_dist  = new Signal<uint16_t>(SIGNAL_PROG_DEST, "sig.prog-dest");
 
    signal_recv_reload = new Signal<uint8_t>("sig.recv-reload");
    signal_prog_send = new Signal<uint8_t>("sig.prog-send");
    signal_prog_recv = new Signal<uint8_t>("sig.prog-recv");
    signal_send_status = new Signal<uint8_t>("sig.send-status");
    signal_recv_status =  new Signal<uint32_t>("sig.recv-status");

    // instantiate modules
    mem = new Memory("main-memory", ORCA_MEMORY_SIZE, ORCA_MEMORY_BASE);
    cpu = new HFRiscV("cpu", signal_intr, signal_stall, mem);

    // map control signals to memory space so that software can access them
    signal_addr->MapTo(mem->GetMap(SIGNAL_PROG_ADDR), SIGNAL_PROG_ADDR);
    signal_size->MapTo(mem->GetMap(SIGNAL_PROG_SIZE), SIGNAL_PROG_SIZE);
    signal_dist->MapTo(mem->GetMap(SIGNAL_PROG_DEST), SIGNAL_PROG_DEST);
    signal_prog_send->MapTo(mem->GetMap(SIGNAL_PROG_SEND), SIGNAL_PROG_SEND);
    signal_prog_recv->MapTo(mem->GetMap(SIGNAL_PROG_RECV), SIGNAL_PROG_RECV);

    // reset control wires
    signal_stall->Write(0);
    signal_intr->Write(0);
    signal_addr->Write(0);
    signal_size->Write(0);
    signal_dist->Write(0);
    
    // create a dma and off-chip comm modules
    bridge = new NetBridge("comm");
    netif = new DmaNetif("dma");

    // connect the dma and netif to each other (via buffers)
    bridge->SetOutputBuffer(netif->GetInputBuffer());
    netif->SetOutputBuffer(bridge->GetInputBuffer());

    // connect netif to control wires
    netif->SetSignalIntr(signal_intr);
    netif->SetSignalStall(signal_stall);
    netif->SetSignalProgAddr(signal_addr);
    netif->SetSignalProgSize(signal_size);
    netif->SetSignalProgDest(signal_dist);
    netif->SetSignalRecvReload(signal_recv_reload);
    netif->SetSignalProgSend(signal_prog_send);
    netif->SetSignalProgRecv(signal_prog_recv);
    netif->SetSignalRecvStatus(signal_recv_status);
    netif->SetSignalSendStatus(signal_send_status);

    // create additional memory for send/recv processes
    mem1 = new Memory("netif-mem-1", ORCA_MEMORY_SIZE_1, 0);
    mem2 = new Memory("netif-mem-2", ORCA_MEMORY_SIZE_2, 0);

    // connect netif to memories
    netif->SetMem0(mem);
    netif->SetMem1(mem1);
    netif->SetMem2(mem2);

    // map counters to memory if hardware counters were enabled
    #ifdef MEMORY_ENABLE_COUNTERS
    // map main memory counter
    mem->GetSignalCounterStore()->MapTo(
        mem->GetMap(M0_COUNTER_STORE_ADDR), M0_COUNTER_STORE_ADDR);
    mem->GetSignalCounterLoad()->MapTo(
        mem->GetMap(M0_COUNTER_LOAD_ADDR), M0_COUNTER_LOAD_ADDR);

    mem1->GetSignalCounterStore()->MapTo(
        mem->GetMap(M1_COUNTER_STORE_ADDR), M1_COUNTER_STORE_ADDR);
    mem1->GetSignalCounterLoad()->MapTo(
        mem->GetMap(M1_COUNTER_LOAD_ADDR), M1_COUNTER_LOAD_ADDR);
    mem2->GetSignalCounterStore()->MapTo(
        mem->GetMap(M2_COUNTER_STORE_ADDR), M2_COUNTER_STORE_ADDR);
    mem2->GetSignalCounterLoad()->MapTo(
        mem->GetMap(M2_COUNTER_LOAD_ADDR), M2_COUNTER_LOAD_ADDR);
    #endif

    #ifdef HFRISCV_ENABLE_COUNTERS
    // memory mapping
    cpu->GetSignalCounterArith()->MapTo(
        mem->GetMap(CPU_COUNTER_ARITH_ADDR), CPU_COUNTER_ARITH_ADDR);
    cpu->GetSignalCounterLogical()->MapTo(
        mem->GetMap(CPU_COUNTER_LOGICAL_ADDR), CPU_COUNTER_LOGICAL_ADDR);
    cpu->GetSignalCounterShift()->MapTo(
        mem->GetMap(CPU_COUNTER_SHIFT_ADDR), CPU_COUNTER_SHIFT_ADDR);
    cpu->GetSignalCounterBranches()->MapTo(
        mem->GetMap(CPU_COUNTER_BRANCHES_ADDR), CPU_COUNTER_BRANCHES_ADDR);
    cpu->GetSignalCounterJumps()->MapTo(
        mem->GetMap(CPU_COUNTER_JUMPS_ADDR), CPU_COUNTER_JUMPS_ADDR);
    cpu->GetSignalCounterLoadStore()->MapTo(
        mem->GetMap(CPU_COUNTER_LOADSTORE_ADDR), CPU_COUNTER_LOADSTORE_ADDR);
    cpu->GetSignalCounterCyclesTotal()->MapTo(
        mem->GetMap(CPU_COUNTER_CYCLES_TOTAL_ADDR),
        CPU_COUNTER_CYCLES_TOTAL_ADDR);
    cpu->GetSignalCounterCyclesStall()->MapTo(
        mem->GetMap(CPU_COUNTER_CYCLES_STALL_ADDR),
        CPU_COUNTER_CYCLES_STALL_ADDR);
    cpu->GetSignalHostTime()->MapTo(
        mem->GetMap(CPU_COUNTER_HOSTTIME_ADDR), CPU_COUNTER_HOSTTIME_ADDR);
    #endif

    // load software image into memory
    mem->LoadBin(GetParam(1), ORCA_MEMORY_BASE, ORCA_MEMORY_SIZE);
}

/**
 * In this method, we add hardware models to the simulation queue. Please note
 * that not all models are added to the queue. See the documentation of proper
 * models to determine whether the model must be scheduled or not. 
 */
void SingleCoreExt::Schedule() {
    // schedule cpu
    // cpu starts at the 3rd cycle due to its the first
    // having one instruction coming out of the pipeline
    Register(this->cpu, 3);
    Register(this->bridge);
    Register(this->netif);
}

void SingleCoreExt::Report() {
    // minimal reporting
    std::cout << "cpu: INTR=" << static_cast<int>(signal_intr->Read())
        << ", STALL=" << static_cast<int>(signal_stall->Read())
        << std::endl;

    // if counters enabled, report them
    #ifdef HFRISCV_ENABLE_COUNTERS
    std::cout << "cpu"
        "\tiarith\t\t" << static_cast<int>(cpu->GetSignalCounterArith()->Read())
        << std::endl <<
        "\tilogic\t\t"
        << static_cast<int>(cpu->GetSignalCounterLogical()->Read())
        << std::endl <<
        "\tishift\t\t" << static_cast<int>(cpu->GetSignalCounterShift()->Read())
        << std::endl <<
        "\tibranch\t\t"
        << static_cast<int>(cpu->GetSignalCounterBranches()->Read())
        << std::endl <<
        "\tijumps\t\t" << static_cast<int>(cpu->GetSignalCounterJumps()->Read())
        << std::endl <<
        "\timemop\t\t"
        << static_cast<int>(cpu->GetSignalCounterLoadStore()->Read())
        << std::endl <<
        "\ticycles\t\t"
        << static_cast<int>(cpu->GetSignalCounterCyclesTotal()->Read())
        << std::endl <<
        "\tistalls\t\t"
        << static_cast<int>(cpu->GetSignalCounterCyclesStall()->Read())
        << std::endl <<
        "\tihtime\t\t" << static_cast<int>(cpu->GetSignalHostTime()->Read())
        << std::endl;

    #endif

    #ifdef MEMORY_ENABLE_COUNTERS
    std::cout << "mem"
        "\tloads\t\t" << static_cast<int>(mem->GetSignalCounterStore()->Read())
        << std::endl <<
        "\tstores\t\t" << static_cast<int>(mem->GetSignalCounterLoad()->Read())
        << std::endl;
    #endif

    SetExitStatus(0);
}

SingleCoreExt::~SingleCoreExt() {
    // free resources
    delete(cpu);
    delete(mem);
    delete(signal_intr);
    delete(signal_stall);  // missing signals here

    delete(mem1);
    delete(mem2);
    delete(netif);
}

/**
 * This is the main routine for your application. This basically instantiates
 * a new simulator and starts the simulation by calling its <Simulate> method.
 */
int main(int argc, char** argv) {
    SingleCoreExt* simulator = new SingleCoreExt(argc, argv);
    simulator->Simulate();
    int ret = simulator->GetExitStatus();
    return ret;
}


    