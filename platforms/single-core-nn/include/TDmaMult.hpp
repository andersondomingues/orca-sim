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
#ifndef PLATFORMS_SINGLE_CORE_NN_INCLUDE_TDMAMULT_HPP_
#define PLATFORMS_SINGLE_CORE_NN_INCLUDE_TDMAMULT_HPP_

// include peripheral addresses
#include "MemoryMap.h"

// std API
#include <iostream>
#include <string>

// simulator API
#include "TimedModel.hpp"
#include "UMemory.hpp"
#include "USignal.hpp"

/*
 The current approach is: the cpu copies data to the DMA memory, configure the DMA, stall the CPU, does MACs in parallel and in burst mode, 
 copies the MAC results to MMIO, release the stall, and the CPU copies the results and cotinue the software.

 An alternative approach, interesting if we have multiple tasks and OS, is not to halt the CPU, letting other tasks using the CPU while
 the DMA is working. When the DMA finishes, and interrupt is asserted, and the results are copied back to the software layer.
*/
enum class DmaState{
    // wait cpu to configure the DMA, indicated by _sig_dma_prog. Then, the DMA
    // raises _sig_stall to stall the cpu while the DMA is working.
    WAIT_CONFIG_STALL,
    // copy content from the NN memory to the MAC internal operand registers.
    COPY_FROM_MEM,
    // just waste a cycle to copy the MAC results to the MMIO.
    COPY_TO_CPU,
    // deassert the _sig_stall, returning to the wait mode.
    // The CPU returns to activity.
    FLUSH
};

/**
 * @class TDmaMult
 * @author Alexandre Amory, based on Anderson's  TDMANetif
 * @date 01/04/20
 * @file TDmaMult.h
 * @brief DMA unit reponsible to transfer weight and input data from the NN memory 
 *	 directly to the vector MACs
 */
class TDmaMult: public TimedModel{
 private:
    // pointer to the main memory
    UMemory* _mem0;
    // base address of the weight memory channel. Once set, it does not change
    // in runtime. it can only be changed in design time.
    uint32_t _memW[SIMD_SIZE];
    // base address of the input memory channel. Once set, it does not change
    // in runtime. it can only be changed in design time.
    uint32_t _memI[SIMD_SIZE];
    // base address to the array with the results from the MAC units. Supposed
    // to be constant. it can only be changed in design time.
    uint32_t _base_mac_out_addr;
    // States for DMA process.
    DmaState _dma_state;

    // control signals.
    // (OUT): stalls cpu while the DMA is copying from the memories.
    USignal<uint8_t>*  _sig_stall;
    // (IN): processor writes 1 to start the DMA.
    USignal<uint8_t>*  _sig_dma_prog;

    // data sent from the processor to program the DMA.
    // IN: number of MACs ops to be executed in burst mode.
    USignal<uint32_t>* _sig_burst_size;
    // IN: (not used) amount of memory configured for each channel.
    // 1 means NN_MEM_SIZE_PER_CHANNEL bytes, 2 means 2*NN_MEM_SIZE_PER
    // CHANNEL bytes, ...
    USignal<uint32_t>* _sig_nn_size;
    // IN: (not used) number of expected output data.
    USignal<uint32_t>* _sig_out_size;

    // internal registers between the pipeline stages.
    // data 'register' pf the 1st pipeline stage, i.e. the operands of the
    // MAC units.
    float _op1[SIMD_SIZE], _op2[SIMD_SIZE];
    // data 'register' between the 2nd and the 3rd pipeline stages.
    // The result of the multiplication.
    float _reg_mul[SIMD_SIZE];
    // data 'register' with the output of the MAC.
    float _reg_mac[SIMD_SIZE];

    /// pipeline signals.
    uint8_t _mul_loaded;  // signal between the 1st and the 2nd pipeline stages.
    uint8_t _mul_ready;  // signal between the 2nd and the 3rd pipeline stages.

    // internal data register. Data sent from the processor to program the DMA.
    uint32_t _burst_size;  // total number of multiplications.
    uint32_t nn_size;  // (not used) number of NN memory banks for a single MAC.
    uint32_t out_size;  // (not used) number of expected output data.
    // others
    uint32_t _remaining;        ///< count number of data to be read.
    /// memory idx used to access both the input and weight memories.
    uint32_t _mem_idx;

    /// Internal processes -- 3 stage pipeline.
    void ReadData();  // 1st pipeline stage, i.e. data fetch
    void DoMult();  // 2nd pipeline stage, multiplication
    void DoAcc();  // 3rd pipeline stage, accumulation

 public:
    // getters
    DmaState GetDmaState();

    // other
    SimulationTime Run();
    void Reset();

    /** ctor
     * @param name: name of the module.
     * @param stall: signal to stall the processor while the DMA is going on.
     * @param burst_size: MMIO with the total number of multiplications.
     * @param nn_size: (not used) number of NN memory banks for a single MAC.
     * @param out_size: (not used) number of expected output data.
     * @param base_mac_out_addr: base address to the array with the output of the MACs.
     * @param mac: pointer to the MAC module.
     * */
    TDmaMult(std::string name, USignal<uint8_t>* stall,
        USignal<uint8_t>* dma_start, USignal<uint32_t>* burst_size,
        USignal<uint32_t>* nn_size, USignal<uint32_t>* out_size,
        uint32_t base_mac_out_addr, UMemory* main_mem);

    /** dtor
     * */
    ~TDmaMult();
};


#endif  // PLATFORMS_SINGLE_CORE_NN_INCLUDE_TDMAMULT_HPP_
