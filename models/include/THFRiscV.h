#ifndef __THFRiscV_H
#define __THFRiscV_H

/**
 * This is file is a modification of the following file
//https://github.com/sjohann81/hellfireos/blob/master/usr/sim/hf_riscv_sim/hf_riscv_sim.c
*/

/* file:          hf_riscv_sim.c
 * description:   HF-RISCV simulator
 * date:          11/2015
 * author:        Sergio Johann Filho <sergio.filho@pucrs.br>
 */

//STD libraries
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

//simulator libs
#include <TProcessorBase.h>

//models libs
#include <UMemory.h>
#include <USignal.h>

#define HFRISCV_PC_MEMBASE 0x40000000

#define EXIT_TRAP			0xe0000000
#define IRQ_VECTOR		0xf0000000
#define IRQ_CAUSE			0xf0000010
#define IRQ_MASK			0xf0000020
#define IRQ_STATUS		0xf0000030
#define IRQ_EPC			0xf0000040
#define COUNTER			0xf0000050
#define COMPARE			0xf0000060
#define COMPARE2			0xf0000070
#define EXTIO_IN			0xf0000080
#define EXTIO_OUT			0xf0000090
#define DEBUG_ADDR		0xf00000d0
#define UART_WRITE		0xf00000e0
#define UART_READ			0xf00000e0
#define UART_DIVISOR		0xf00000f0


typedef struct {
	int32_t r[32];
	uint32_t pc, pc_next;
	
	uint32_t vector, cause, mask, status, status_dly[4], epc, counter, compare, compare2;
	uint64_t cycles;
} risc_v_state;

//inherits for a 32-bit processor
class THFRiscV : public TProcessorBase<uint32_t>{

private:
	uint32_t _last_pc;

	//interruption wire
	USignal<uint8_t>* _signal_intr;
	USignal<uint8_t>* _signal_stall;
	
	//context
	risc_v_state* s;

	int i;
	
	#ifdef HFRISCV_ENABLE_COUNTERS
	USignal<uint32_t>* _counter_iarith;
	USignal<uint32_t>* _counter_ilogical;
	USignal<uint32_t>* _counter_ishift;
	USignal<uint32_t>* _counter_ibranches;
	USignal<uint32_t>* _counter_ijumps;
	USignal<uint32_t>* _counter_iloadstore;
	
	USignal<uint32_t>* _counter_cycles_total;
	USignal<uint32_t>* _counter_cycles_stall;
	
	USignal<uint32_t>* _counter_hosttime;
	#endif
	
public:

	#ifdef HFRISCV_ENABLE_COUNTERS
	USignal<uint32_t>* GetSignalCounterArith();
	USignal<uint32_t>* GetSignalCounterLogical();
	USignal<uint32_t>* GetSignalCounterShift();
	USignal<uint32_t>* GetSignalCounterBranches();
	USignal<uint32_t>* GetSignalCounterJumps();
	USignal<uint32_t>* GetSignalCounterLoadStore();
	
	USignal<uint32_t>* GetSignalCounterCyclesTotal();
	USignal<uint32_t>* GetSignalCounterCyclesStall();
	
	USignal<uint32_t>* GetSignalHostTime();
	
	void InitCounters(
		uint32_t arith_counter_addr, 
		uint32_t logical_counter_addr,
		uint32_t shift_counter_addr,
		uint32_t branches_counter_addr,
		uint32_t jumps_counter_addr, 
		uint32_t loadstore_counter_addr,
		uint32_t cycles_total_counter_addr, 
		uint32_t cycles_stall_counter_addr,
		uint32_t hosttime_addr);
	
	void UpdateCounters(int opcode, int func3);
	#endif

    //risc_v_state GetState();
    
	void dumpregs(risc_v_state *s);
	void bp(risc_v_state *s, uint32_t ir);
	int32_t mem_read(risc_v_state *s, int32_t size, uint32_t address);
	void mem_write(risc_v_state *s, int32_t size, uint32_t address, uint32_t value);

	//ctor./dtor.
	THFRiscV(string name, USignal<uint8_t>* intr, USignal<uint8_t>* stall, UMemory* mem);
	~THFRiscV();
	
	USignal<uint8_t>* GetSignalStall();
	USignal<uint8_t>* GetSignalIntr();

	SimulationTime Run();
	
	//file output
	ofstream output_debug;
	ofstream output_uart;
	
   void Reset();	
};

#endif /* __RISC_V_H */
