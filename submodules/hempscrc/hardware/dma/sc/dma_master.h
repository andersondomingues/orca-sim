#ifndef _DMA_MASTER_h
#define _DMA_MASTER_h

#include <systemc.h>

SC_MODULE(dma_master){
    sc_in<bool>					clock;
    sc_in<bool>					reset;
    sc_in<bool>					read_av;
    sc_out<bool>				read_data;
    sc_in<bool>					send_av;
    sc_out<bool>				send_data;
    sc_in<bool>					set_address;
    sc_in<bool>					set_size;
    sc_in<bool>					set_op;
    sc_in<bool>					start;
    sc_in<sc_uint<32> >			data_read;
    sc_out<sc_uint<32> >		data_write;
    sc_out<bool>				active;
    sc_out<bool>				intr;
    sc_in<bool>					intr_ack;
    sc_in<bool>					data_valid;
    sc_out<sc_uint<32> >		mem_address;
    sc_out<sc_uint<32> >		mem_addr_ddr;
    sc_out<bool>				mem_ddr_read_req;
    sc_out<bool>				mem_ddr_access;
    sc_out<sc_uint<32> >		mem_data_write;
    sc_in<sc_uint<32> >			mem_data_read;
    sc_out<sc_uint<4> >			mem_byte_we;
	
	enum state					{SWait, SWaitMem0, SWaitMem1, SCopyFromMem, SCopyFromRam, SCopyFromRam0, SCopyToMem, SEnd, SCopyFromRamWait, SCopyFromRamWait1};
	sc_signal<state >			EA;

	sc_signal<sc_uint<32> >				address;
	sc_signal<sc_uint<32> >				data_reg;
	sc_signal<sc_uint<32> >				data_write_reg;//Deals with the one cycle memory latency
	sc_signal<sc_uint<16> >				size;
	sc_signal<bool >			operation;
	sc_signal<bool >			read_data_reg;

	sc_signal<sc_uint<32> >				mem_data_read_reg;
	sc_signal<bool >			data_valid_reg;
	sc_signal<bool >			send_data1;

	void dma_fsm();
	void comb_assign();
	
	SC_CTOR(dma_master){
		SC_METHOD(dma_fsm);
		sensitive << clock.pos();
		sensitive << reset;
		
		SC_METHOD(comb_assign);
		sensitive << address;
		sensitive << operation;
		sensitive << data_reg;
		sensitive << read_data_reg;
		sensitive << mem_data_read;
		sensitive << EA;
		sensitive << read_av;
		sensitive << send_av;
    }
};

#endif

