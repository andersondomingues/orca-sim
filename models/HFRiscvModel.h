#ifndef __RISC_V_H
#define __RISC_V_H

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
#include <Process.h>

//models libs
#include <MemoryModel.h>

#define MEM_SIZE			0x00100000
#define EXIT_TRAP			0xe0000000
#define IRQ_VECTOR			0xf0000000
#define IRQ_CAUSE			0xf0000010
#define IRQ_MASK			0xf0000020
#define IRQ_STATUS			0xf0000030
#define IRQ_EPC				0xf0000040
#define COUNTER				0xf0000050
#define COMPARE				0xf0000060
#define COMPARE2			0xf0000070
#define EXTIO_IN			0xf0000080
#define EXTIO_OUT			0xf0000090
#define DEBUG_ADDR			0xf00000d0
#define UART_WRITE			0xf00000e0
#define UART_READ			0xf00000e0
#define UART_DIVISOR		0xf00000f0

#define ntohs(A) ( ((A)>>8) | (((A)&0xff)<<8) )
#define htons(A) ntohs(A)
#define ntohl(A) ( ((A)>>24) | (((A)&0xff0000)>>8) | (((A)&0xff00)<<8) | ((A)<<24) )
#define htonl(A) ntohl(A)

typedef struct {
	int32_t r[32];
	uint32_t pc, pc_next;
	MemoryType* mem;
	uint32_t vector, cause, mask, status, status_dly[4], epc, counter, compare, compare2;
	uint64_t cycles;
} risc_v_state;

class HFRiscvModel : public Process{

private:
	//when exit trap is triggered, activate 
	//this flag to remove the cpu from simulation
	//scheduling
	bool _disabled; 
	
		risc_v_state context;
		risc_v_state *s;
		int i;		

        //out wires
        uint32_t _mem_address; 
        uint32_t _mem_data_write;
        bool _mem_pause;
        uint8_t _current_page; //can be ignored for riscv        
        
        //in wires
        bool* _intr_in;
        uint32_t* _mem_data_read;
        uint8_t * _byte_we;

public:
	
        
		void dumpregs(risc_v_state *s);
		void bp(risc_v_state *s, uint32_t ir);
		int32_t mem_fetch(risc_v_state *s, uint32_t address);
		int32_t mem_read(risc_v_state *s, int32_t size, uint32_t address);
		void mem_write(risc_v_state *s, int32_t size, uint32_t address, uint32_t value);

		HFRiscvModel(string name, MemoryType* mptr, uint32_t size, uint32_t base);
		unsigned long long Run();
		unsigned long long cycle(risc_v_state *s);
		
		ofstream output_debug, output_uart;
		
		~HFRiscvModel();
    
        /**
         * @brief Processor reset.*/
        void Reset();        
};

#endif /* __RISC_V_H */
