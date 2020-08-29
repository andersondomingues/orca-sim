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
#ifndef MODELS_HFRISCV_CORE_INCLUDE_HFRISCV_HPP_
#define MODELS_HFRISCV_CORE_INCLUDE_HFRISCV_HPP_

// std libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <string>

// simulator libs
#include "ProcessorBase.hpp"

// models libs
#include "Memory.hpp"
#include "Signal.hpp"

#define HFRISCV_PC_MEMBASE 0x40000000

#define EXIT_TRAP       0xe0000000
#define IRQ_VECTOR      0xf0000000
#define IRQ_CAUSE       0xf0000010
#define IRQ_MASK        0xf0000020
#define IRQ_STATUS      0xf0000030
#define IRQ_EPC         0xf0000040
#define COUNTER         0xf0000050
#define COMPARE         0xf0000060
#define COMPARE2        0xf0000070
#define EXTIO_IN        0xf0000080
#define EXTIO_OUT       0xf0000090
#define DEBUG_ADDR      0xf00000d0
#define UART_WRITE      0xf00000e0
#define UART_READ       0xf00000e0
#define UART_DIVISOR    0xf00000f0

using orcasim::modeling::ProcessorBase;
using orcasim::modeling::Signal;
using orcasim::modeling::Memory;

namespace orcasim::models::hfriscv {

typedef struct {
    int32_t r[32];
    uint32_t vector, cause, mask, status, status_dly[4], epc,
        counter, compare, compare2;
    uint64_t cycles;
} risc_v_state;

// inherits for a 32-bit processor
class HFRiscV : public ProcessorBase<uint32_t>{
 private:
    uint32_t _last_pc;

    // interruption wire
    Signal<uint8_t>* _signal_intr;
    Signal<uint8_t>* _signal_stall;

    // context
    risc_v_state* s;

    int i;

    #ifdef HFRISCV_ENABLE_COUNTERS
    void UpdateCounters(int opcode, int funct3);

    Signal<uint32_t>* _counter_iarith;
    Signal<uint32_t>* _counter_ilogical;
    Signal<uint32_t>* _counter_ishift;
    Signal<uint32_t>* _counter_ibranches;
    Signal<uint32_t>* _counter_ijumps;
    Signal<uint32_t>* _counter_iloadstore;

    Signal<uint32_t>* _counter_cycles_total;
    Signal<uint32_t>* _counter_cycles_stall;

    Signal<uint32_t>* _counter_hosttime;
    #endif

 public:
    #ifdef HFRISCV_ENABLE_COUNTERS
    Signal<uint32_t>* GetSignalCounterArith();
    Signal<uint32_t>* GetSignalCounterLogical();
    Signal<uint32_t>* GetSignalCounterShift();
    Signal<uint32_t>* GetSignalCounterBranches();
    Signal<uint32_t>* GetSignalCounterJumps();
    Signal<uint32_t>* GetSignalCounterLoadStore();

    Signal<uint32_t>* GetSignalCounterCyclesTotal();
    Signal<uint32_t>* GetSignalCounterCyclesStall();

    Signal<uint32_t>* GetSignalHostTime();
    #endif

    void dumpregs();
    void bp(risc_v_state *s, uint32_t ir);
    int32_t mem_read(risc_v_state *s, int32_t size, uint32_t address);
    void mem_write(risc_v_state *s, int32_t size, uint32_t address,
        uint32_t value);

    // ctor./dtor.
    HFRiscV(std::string name, Signal<uint8_t>* intr, Signal<uint8_t>* stall,
        Memory* mem);
    ~HFRiscV();

    Signal<uint8_t>* GetSignalStall();
    Signal<uint8_t>* GetSignalIntr();

    SimulationTime Run();

    // file output
    std::ofstream output_debug;
    std::ofstream output_uart;

    void Reset();
};

}  // namespace orcasim::models::hfriscv
#endif  // MODELS_HFRISCV_CORE_INCLUDE_HFRISCV_HPP_
