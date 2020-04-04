/** 
 * This file is part of project URSA. More information on the project
 * can be found at 
 *
 * URSA's repository at GitHub: http://https://github.com/andersondomingues/ursa
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
#ifndef __TDMAMULT_H
#define __TDMAMULT_H

//std API
#include <iostream>

//simulator API
#include <TimedModel.h>
#include <UMemory.h>
#include <USignal.h>
#include <TMult.h>

// MMIO registers
//0x40410000 => memory mapped control wires
#define SIGNAL_CPU_STALL    0x40410000  // 8 bits
#define SIGNAL_CPU_INTR     0x40410001
//#define SIGNAL_DMA_STATUS   0x40410002
//#define SIGNAL_DMA_PROG     0x40410003  //  TODO is it required ?!?!

// jumping to 0x40420000, otherwise it wont fit before the memory counters
#define DMA_BURST_SIZE       0x40420000  // 32 bits 
#define DMA_WEIGHT_MEM_ADDR  0x40420004
#define DMA_INPUT_MEM_ADDR   0x40420008  
#define DMA_MAC_OUT          0x4042000C

enum class DmaState{
	WAIT_CONFIG_STALL, //wait cpt to configure and raise _sig_send, stall
	COPY_FROM_MEM,     //copy content from memory, release cpu
	FLUSH              //wait for the cpu to lower the send signal (ack)
};

/**
 * @class TDmaMult
 * @author Alexandre Amory, based on Anderson's  TDMANetif
 * @date 01/04/20
 * @file TDmaMult.h
 * @brief DMA unit reponsible to transfer weight and input data from the main memory 
 *	 directly to the vector multipliers
 */
class TDmaMult: public TimedModel{

private:

	/// Pointer to weight memory.
	UMemory* _memW;
	/// Pointer to input memory .
	UMemory* _memI;
	/// Pointer to the MAC.
	TimedFPMultiplier* _mult;
	/// States for DMA process.
    DmaState _dma_state;
    
	///@{
	/// control signals.
	USignal<uint8_t>*  _sig_stall;      ///< (OUT): stalls cpu while the DMA is copying from the memories.
	///@}

	///@{
    /// data sent from the processor to program the DMA.
	USignal<uint32_t>* _sig_burst_size;       ///< IN: total number of multiplications.
	USignal<uint32_t>* _sig_weight_mem_addr;  ///< IN: initial address of the weight memory.
	USignal<uint32_t>* _sig_input_mem_addr;   ///< IN: initial address of the input memory.
	///@}

	///@{
    /// data read by the processor after the interruption.
	/// OUT: register with the final result from the MAC, to be read by the processor.
	USignal<uint32_t>* _sig_mac_out;      
	///@}

	///@{
    /// internal registers between the pipeline stages.
	uint32_t  _reg_mul;  ///< data 'register' between the 2nd and the 3rd pipeline stages. The result of the multiplication.
	uint32_t  _reg_mac;  ///< data 'register' with the output of the MAC.
	///@}

	///@{
	/// pipeline signals.
	uint8_t _mul_loaded; ///< signal between the 1st and the 2nd pipeline stages.
	uint8_t _mul_ready;  ///< signal between the 2nd and the 3rd pipeline stages.
	///@}

	//data sent from the processor to program the DMA
	uint32_t _burst_size;       //total number of multiplications
	uint32_t _weight_mem_addr;  //initial address of the weight memory
	uint32_t _input_mem_addr;   //initial address of the input memory	

	//others 
	uint32_t _remaining;        //number of data to be read

public:	
    
    //getters
    //DmaNetifRecvState GetRecvState();
	DmaState GetDmaState();
    
    //getters
    USignal<uint8_t>*  GetSignalStall();
	USignal<uint8_t>*  GetSignalIntr();
	USignal<uint8_t>*  GetSignalDmaStatus();
	USignal<uint8_t>*  GetSignalDmaProg();

	USignal<uint32_t>* GetSignalBurstSize();
	USignal<uint32_t>* GetSignalWeightMemAddr();
	USignal<uint32_t>* GetSignalInputMemAddr();
	USignal<uint32_t>* GetSignalMacOut();

	//setters
    void SetSignalStall(USignal<uint8_t>*);
	void SetSignalIntr(USignal<uint8_t>*);
	void SetSignalDmaStatus(USignal<uint8_t>*);
	void SetSignalDmaProg(USignal<uint8_t>*);

	void SetSignalBurstSize(USignal<uint32_t>*);
	void SetSignalWeightMemAddr(USignal<uint32_t>*);
	void SetSignalInputMemAddr(USignal<uint32_t>*);
	void SetSignalMacOut(USignal<uint32_t>*);

    //internal processes -- 3 stage pipeline
    void ReadData();
	void DoMult();
	void DoAcc();

    //other 
    SimulationTime Run();
    void Reset();

	//memories
	void SetMemW(UMemory*);
	void SetMemI(UMemory*);

	//mult
	void SetMult(TimedFPMultiplier*);

    /** ctor
     * @param name: name of the module.
	 * @param stall: signal to stall the processor while the DMA is going on.
     * @param burst_size: MMIO with the total number of multiplications.
     * @param weight_mem_addr: MMIO with the initial address of the weight memory.
	 * @param input_mem_addr: MMIO with the initial address of the input memory.
	 * @param mac_out: MMIO with the register with the final result from the MAC, to be read by the processor.
	 * @param memW: pointer to the base address of the weight memory bank.
	 * @param memI: pointer to the base address of the input memory bank.
	 * @param mac: pointer to the MAC module.
	 * */
    TDmaMult(string name, USignal<uint8_t>* stall, USignal<uint32_t>* burst_size, 
		USignal<uint32_t>* weight_mem_addr, USignal<uint32_t>* input_mem_addr, USignal<uint32_t>* mac_out,
		UMemory* memW, UMemory* memI, TimedFPMultiplier* mac);
		
	/** dtor
	 * */
    ~TDmaMult();
};


#endif /* __TDMAMULT_H */
