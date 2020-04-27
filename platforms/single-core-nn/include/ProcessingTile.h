/** 
 * This file is part of project URSA. More information on the project
 * can be found at URSA's repository at GitHub
 * 
 * http://https://github.com/andersondomingues/ursa
 *
 * Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. **/
#ifndef __ProcessingTile_H
#define __ProcessingTile_H

//std API
#include <iostream>

//model API
#include <THellfireProcessor.h>
#include <UMemory.h>
#include <USignal.h>
#include <TDmaMult.h>

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

//main memory mapping
#define MEM0_SIZE 0x008FFFFF 
#define MEM0_BASE 0x40000000

//->>>> first available address for memory mapping 0x40410000
// DMA MMIO registers
// jumping to 0x404120xx, otherwise it wont fit before the memory counters
#define SIGNAL_CPU_STALL    0x40412000  // 8 bits
#define SIGNAL_CPU_INTR     0x40412001
#define SIGNAL_DMA_PROG     0x40412002
//							0x40410003  // not used
#define DMA_BURST_SIZE	    0x40412004  // 32 bits 
#define DMA_NN_SIZE  	    0x40412008
#define DMA_OUT_SIZE  	    0x4041200C
#define DMA_MAC_OUT_ARRAY   0x40412010
// DMA_MAC_OUT_ARRAY0 		0x40412010
// DMA_MAC_OUT_ARRAY1 		0x40412014
//...  
//DMA_MAC_OUT_ARRAY31		0x40412010 + (0x20 * 4) -1 

//0x40411xxx => memory mapped counters
#ifdef MEMORY_ENABLE_COUNTERS
#define M0_COUNTER_STORE_ADDR (0x40411010)
#define M0_COUNTER_LOAD_ADDR  (0x40411014)
#define M1_COUNTER_STORE_ADDR (0x40411018)
#define M1_COUNTER_LOAD_ADDR  (0x4041101C)
#define M2_COUNTER_STORE_ADDR (0x40411020)
#define M2_COUNTER_LOAD_ADDR  (0x40411024)
#endif

#ifdef HFRISCV_ENABLE_COUNTERS
#define CPU_COUNTER_ARITH_ADDR     (0x40411128)
#define CPU_COUNTER_LOGICAL_ADDR   (0x4041112C)
#define CPU_COUNTER_SHIFT_ADDR     (0x40411130)
#define CPU_COUNTER_BRANCHES_ADDR  (0x40411134)
#define CPU_COUNTER_JUMPS_ADDR     (0x40411138)
#define CPU_COUNTER_LOADSTORE_ADDR (0x4041113C)
#define CPU_COUNTER_HOSTTIME_ADDR  (0x40411140)
#define CPU_COUNTER_CYCLES_TOTAL_ADDR (0x40411144)
#define CPU_COUNTER_CYCLES_STALL_ADDR (0x40411148)
#endif

//0x403F15xx => router wires
#ifdef ROUTER_ENABLE_COUNTERS
#define ROUTER_COUNTER_ACTIVE_ADDR (0x40411500)
#endif

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

	///@{ Main components of the system.
	/// the hfrisv-core.
	THellfireProcessor* _cpu; 
	/// the main memory.
	UMemory* _mem0;
	/// DMA unit reponsible to transfer weight and input data from the main memory directly to the vector multipliers
	TDmaMult* _dma;
	///@}

	//hosttime magic wire
	uint32_t _shosttime;
	USignal<uint32_t>* _signal_hosttime;
	
	///@{
	/// control signals.
	USignal<uint8_t>*  _sig_stall;          ///< stalls cpu while copying from the memories
	USignal<uint8_t>*  _sig_dma_prog;       ///< flag to start the DMA
	USignal<uint8_t>*  _sig_intr;			///< dummy signal required by the CPU. not really used since we dont have interrupts in this design
	///@}
	
	///@{
	/// data sent from the processor to program the DMA.
	USignal<uint32_t>* _sig_burst_size;		///< number of MACs ops to be executed in burst mode.
	USignal<uint32_t>* _sig_nn_size;  		///< (not used) amount of memory configured for each channel. 1 means NN_MEM_SIZE_PER_CHANNEL bytes, 2 means 2*NN_MEM_SIZE_PER_CHANNEL bytes, ...
	USignal<uint32_t>* _sig_out_size;   	///< (not used) number of expected output data. 
	///@}

public: 
	ProcessingTile();
	~ProcessingTile();
	
	//getters
    USignal<uint8_t>*  GetSignalStall();
	USignal<uint8_t>*  GetSignalDmaProg();
	USignal<uint8_t>*  GetSignalIntr();    ///> required only by the cpu and orca. not really usefull 
	
	//getters
	UMemory* GetMem0();
	/// getters for the scheduler
	THellfireProcessor* GetCpu();
	TDmaMult* GetDma();
	
	/**
	 * @brief Get current signal for systime signal
	 * @return A pointer to the instance of signal
	 */
	USignal<uint32_t>* GetSignalHostTime();
	
	std::string ToString();
	std::string GetName();

	void Reset();
};


#endif /* TROUTER_H */
