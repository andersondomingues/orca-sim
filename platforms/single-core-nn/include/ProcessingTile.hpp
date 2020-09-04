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
#ifndef PLATFORMS_SINGLE_CORE_NN_INCLUDE_PROCESSINGTILE_HPP_
#define PLATFORMS_SINGLE_CORE_NN_INCLUDE_PROCESSINGTILE_HPP_

// std API
#include <iostream>
#include <string>

// model API
#include "THFRiscV.hpp"
#include "UMemory.hpp"
#include "USignal.hpp"
#include "TDmaMult.hpp"

/* MEMORY LAYOUT
------------------- 0x40000000 <<-- code begin

       sram
      (4MBytes)

------------------- 0x40400000 <<-- stack
     empty space 
     (64KBytes)
------------------- 0x40410000 <<-- mmio begin
       mmio
     (1Mbytes - 64KBytes)
    
    0x40410000 => various MMIO registers
    0x40411xxx => MMIO performance counters
    0x404120xx => mmio NN DMA
    0x40412100 until 0x404FFFFF => available

------------------- 0x404FFFFF <<-- mmio end


------------------- 0x40500000 <<-- mmio NN MEM banks

    NN MEM banks
     (TOTAL_NN_MEM_SIZE Bytes) // # TOTAL_NN_MEM_SIZE/2 Bytes  for weight and TOTAL_NN_MEM_SIZE/2 Bytes for inputs
     (4MBytes)

------------------- 0x408FFFFF <<-- mmio NN MEM banks 

total memory space: 9MBytes

------------------- 0x404120xx <<-- mmio NN DMA
since there are up to 16 MACs, it is neceseray to reserve
4 32-bit resgisters X 16 MACs = 4 x 4 x 16 = 256 bytes of MMIO for the NN DMA
    0x40412000 DMA0
    0x40412010 DMA1
    0x40412020 DMA2
    ...
    0x404120F0 DMA15
------------------- 0x40500000 <<-- mmio NN MEM banks


max TOTAL_NN_MEM_SIZE = NN_MEM_SIZE_PER_CHANNEL * 2 * SIMD_SIZE. 
It means that the max size for weight is 2MBytes. the same size for inputs.



NN_TOTAL_MEM_HEIGHT determines the total number of words (32bits) of 
the weight and the input memories. These two memories feed the MAC Units.
Assuming NN_TOTAL_MEM_HEIGHT = 1024 (4KBytes), the memory map is:
    0x40500000 - uint32_t weight[NN_TOTAL_MEM_HEIGHT]
    0x40700000 - uint32_t input[NN_TOTAL_MEM_HEIGHT]

In fact, the weight and input is divided into individual banks such that 
it is possible to load each MACs in parallel. Since SIMD_SIZE tells the 
# of MACs running in parallel, then the actual memory map per bank is:
    0x40500000 - uint32_t weight[SIMD_SIZE][NN_TOTAL_MEM_HEIGHT/SIMD_SIZE]
    0x40700000 - uint32_t input[SIMD_SIZE][NN_TOTAL_MEM_HEIGHT/SIMD_SIZE]

where, for instance:
    -  weight[0][0]  is the 1st address of the MAC0
    -  weight[15][0] is the 1st address of the MAC15
*/

// main memory mapping
// #define MEM0_SIZE 0x008FFFFF
// #define MEM0_BASE 0x40000000
#define MEM0_SIZE ORCA_MEMORY_SIZE
#define MEM0_BASE ORCA_MEMORY_BASE

#include <MemoryMap.h>

/**
 * @class ProcessingTile
 * @author Alexandre Amory
 * @date 01/04/20
 * @file ProcessingTile.h
 * @brief This class models an entire processing element that contains
 * RAM memory, DMA for the SIMD vector unit,  HFRiscV core
 */
class ProcessingTile{
 private:
    // the hfrisv-core.
    HFRiscV* _cpu;
    // the main memory.
    Memory* _mem0;
    // DMA unit reponsible to transfer weight and input data from the main
    // memory directly to the vector multipliers
    TDmaMult* _dma;

    // hosttime magic wire
    uint32_t _shosttime;
    Signal<uint32_t>* _signal_hosttime;

    // control signals.
    // stalls cpu while copying from the memories
    Signal<uint8_t>*  _sig_stall;

    // flag to start the DMA
    Signal<uint8_t>*  _sig_dma_prog;

    // dummy signal required by the CPU. not really used since we dont have
    // interrupts in this design
    Signal<uint8_t>*  _sig_intr;

    // data sent from the processor to program the DMA.
    // number of MACs ops to be executed in burst mode.
    Signal<uint32_t>* _sig_burst_size;

    // (not used) amount of memory configured for each channel.
    // 1 means NN_MEM_SIZE_PER_CHANNEL bytes,
    // 2 means 2*NN_MEM_SIZE_PER_CHANNEL bytes, ...
    Signal<uint32_t>* _sig_nn_size;

    // (not used) number of expected output data.
    Signal<uint32_t>* _sig_out_size;

 public:
    ProcessingTile();
    ~ProcessingTile();

    // getters
    Signal<uint8_t>*  GetSignalStall();
    Signal<uint8_t>*  GetSignalDmaProg();
    // required only by the cpu and orca. not really usefull
    Signal<uint8_t>*  GetSignalIntr();

    // getters
    Memory* GetMem0();
    /// getters for the scheduler
    HFRiscV* GetCpu();
    TDmaMult* GetDma();

    /**
     * @brief Get current signal for systime signal
     * @return A pointer to the instance of signal
     */
    Signal<uint32_t>* GetSignalHostTime();

    std::string ToString();
    std::string GetName();

    void Reset();
};


#endif  // PLATFORMS_SINGLE_CORE_NN_INCLUDE_PROCESSINGTILE_HPP_
