#ifndef _Network_Interface_h
#define _Network_Interface_h

#include <systemc.h>
#include "../../router/sc/packet.h"

#define BUFF_SIZE 16

SC_MODULE(Network_Interface){

	sc_in<bool > 				clock;
	sc_in<bool > 				reset;

	// NoC Interface (Local port)    
	sc_out<bool > 				tx;
	sc_out<regflit > 			data_out;
	sc_in<bool > 				credit_i;       
	sc_out<bool > 				clock_tx; 
	sc_in<bool > 				rx;
	sc_in<regflit >				data_in;
	sc_out<bool > 				credit_o;
	sc_in<bool > 				clock_rx;

	// Plasma Interface
	sc_out<bool > 				hold;     
	sc_out<bool > 				send_av;
	sc_out<bool > 				read_av;
	sc_out<bool > 				intr;
	sc_in<bool > 				send_data;
	sc_in<bool > 				read_data;     
	sc_in<sc_uint<32> > 		data_write;
	sc_out<sc_uint<32> > 		data_read;      
	sc_out<sc_uint<32> > 		config;
	
	//Send block signals
	enum state_send 			{S0, S1, S2, S3};
	sc_signal<state_send>		SS;
	
	sc_signal<regflit >			low_word; 		// Stores a low word flit
	sc_signal<regflit > 		payload_size; 	// Stores the payload NoC packet size
	sc_signal<bool > 			tx_reg;
	
	//Receive block signals
	enum state_receive			{SR0, SR1, SR2, SR3, SR4, SR5};
	sc_signal<state_receive >	SR, SP;
							
	sc_signal<sc_uint<4> > 		first, last;
	sc_signal<sc_uint<33> > 	buffer_write, buffer_read;
	sc_signal<bool > 			we;
	sc_signal<bool > 			slot_available;
	sc_signal<bool >			header_stored;			// Store a header flit
	sc_signal<regflit > 		reg_header;				// Indicates a header flit stored
	//sc_signal<bool >			read_av_reg;			// Output registred			
	sc_signal<regflit >			payload_size_receive;	// Counter
	sc_signal<bool > 			payload_size_readed;	// Indicates a payload_size stored
	
	//littleBlock *(myLittleBlock[33]);
	sc_signal<sc_uint<33> >		buffer[BUFF_SIZE];
	
	void config_update();
	void send_av_update();
	void tx_update();
	void clock_tx_update();
	void send_process();
	//void data_read_update();
	void credit_o_update();
	void receive_process();
	//void read_av_update();
	void buffer_read_control();
	
	void w_buffer();
    void r_buffer();
    
	SC_HAS_PROCESS(Network_Interface);
	Network_Interface(sc_module_name name_, regmetadeflit address_router_ = 0) :
	sc_module(name_), address_router(address_router_)
	{
		//lb_buff = INIT;
		
		SC_METHOD(config_update);
		sensitive << clock.pos();
		
		SC_METHOD(send_av_update);
		sensitive << SS;
		sensitive << credit_i;
		
		SC_METHOD(tx_update);
		sensitive << tx_reg;
		
		SC_METHOD(clock_tx_update);
		sensitive << clock.pos(); 
		
		SC_METHOD(send_process);
		sensitive << clock.pos();
		sensitive << reset;
		
		//SC_METHOD(data_read_update);
		//sensitive << buffer_read;
		
		SC_METHOD(credit_o_update);
		sensitive << slot_available;
		
		SC_METHOD(receive_process);
		sensitive << clock.pos();
		sensitive << reset;
		
		//SC_METHOD(read_av_update);
		//sensitive << read_av_reg;
		
		SC_METHOD(buffer_read_control);
		sensitive << clock.pos();
		sensitive << reset;
		
		SC_METHOD(w_buffer);
		sensitive << we;
		sensitive << last;
		
		SC_METHOD(r_buffer);
		sensitive << first;
		for(int i=0;i<BUFF_SIZE;i++){
			sensitive << buffer[i];
		}
	
	}
	private:
		regmetadeflit address_router;
};

#endif
