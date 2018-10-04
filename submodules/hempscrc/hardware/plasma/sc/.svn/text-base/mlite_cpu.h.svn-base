/***
*	Mlite ISS (C)
*	Microprocessor Interface (SystemC)
*	Everton Alceu Carara (carara@inf.pucrs.br)
***/

#ifndef _mlite_cpu_h
#define _mlite_cpu_h

#include <systemc.h>

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
//#include "../../../../software/build/ids_slave.h"	/* PAGESIZE */
#define PAGESIZE 32768


typedef struct {
   long int r[32];
   unsigned long int pc, epc;
   long int hi;
   long int lo;
} State;


SC_MODULE(mlite_cpu) {

	sc_in< bool > clk;
	sc_in< bool > reset_in;
	sc_in< bool > intr_in;
	sc_in< bool > mem_pause;

	sc_out< sc_uint<32> > mem_address;
	sc_out< sc_uint<32> > mem_data_w;
	sc_in < sc_uint<32> > mem_data_r;
	sc_out< sc_uint<4> >  mem_byte_we;

	sc_out< sc_uint<8> >  current_page;


	State *state, state_instance;

	unsigned int opcode, prefetched_opcode, pc_last;
	unsigned int op, rs, rt, rd, re, func, imm, target;
	int imm_shift;
	int *r, word_addr;
	unsigned int *u;
	unsigned int ptr, page, byte_write;
	unsigned char big_endian, shift;
	sc_uint<4> byte_en;

	bool intr_enable, prefetch, jump_or_branch, no_execute_branch_delay_slot;


	SC_CTOR(mlite_cpu) {

		//SC_CTHREAD(cycle,clk.pos());
		SC_THREAD(mlite);
		sensitive << clk.pos() << mem_pause.pos();
		sensitive << mem_pause.neg();

		//printf("\nMlite ISS.\n\n");

		state = &state_instance;

		// MIPS: Big endian.
		big_endian = 1;

		// Used to generate the 'current_page' signal from the 'page' signal
		shift = log10(PAGESIZE)/log10(2);
	}

	/*** Process function ***/
	void mlite();

	/*** Helper functions ***/
	void mult_big(unsigned int a, unsigned int b);
	void mult_big_signed(int a, int b);
};

#endif
