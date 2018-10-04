#ifndef _access_repository_h
#define _access_repository_h

#include <systemc.h>

SC_MODULE(access_repository){

	sc_in<bool > 			clock;
	sc_in<bool > 			reset;
	
	//access to repository in ddr2
	sc_out<bool > 			read_req;
	sc_out<sc_uint<30> >	address;
	sc_in<bool >			data_valid;
	sc_in<sc_uint<32> >		data_read;
	//dma acess
	sc_in<bool >			mem_ddr_access;
	sc_in<sc_uint<32> >		dma_mem_addr_ddr;
	sc_in<bool >			dma_mem_ddr_read_req;
	//plasma interface
	sc_in<sc_uint<32> >		cpu_mem_address;
	sc_in<sc_uint<32> >		cpu_mem_address_reg;
	sc_out<bool >			mem_hold;
	sc_out<sc_uint<32> >	data_read_reg;
	
	//Receive block signals
	enum state {wait_addr, set_req, wait_data, set_done, dma_access};
	sc_signal<state >	ea, pe;
							
	void fsm_sequ_assign();
	void fsm_comb_assign();
	
	SC_CTOR(access_repository) {
		
		SC_METHOD(fsm_sequ_assign);
		sensitive << reset.pos();
		sensitive << clock.pos();
		
		SC_METHOD(fsm_comb_assign);
		sensitive << ea;
		sensitive << cpu_mem_address;
		sensitive << data_valid;
		sensitive << mem_ddr_access;
	}

};

#endif
