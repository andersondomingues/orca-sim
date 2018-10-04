#ifndef _switch_cc_h
#define _switch_cc_h

#include <systemc.h>
#include "packet.h"
#include "queue.h"
#include "switchcontrol.h"

SC_MODULE(router_cc){

  sc_in<bool >			clock;
  sc_in<bool >			reset_n;
  
  sc_in<bool > 			clock_rx[NPORT];
  sc_in<bool >			rx[NPORT];
  sc_out<bool >			credit_o[NPORT];
  sc_out<regflit >		data_out[NPORT];

  sc_out<bool>			clock_tx[NPORT];
  sc_out<bool >			tx[NPORT];
  sc_in<bool >			credit_i[NPORT];
  sc_in<regflit >		data_in[NPORT];
  
  //sc_signal<regNport >	h, ack_h, data_av, sender, data_ack, data_av_ant;

  sc_signal<bool >		sgn_h[NPORT];
  sc_signal<bool >		sgn_ack_h[NPORT];
  sc_signal<bool >		sgn_data_av[NPORT];
  sc_signal<bool >		sgn_sender[NPORT];
  sc_signal<bool >		sgn_data_ack[NPORT];
  sc_signal<reg3 >		incoming;
  sc_signal<regflit >	data[NPORT];
  
  sc_signal<reg_mux >	mux_in, mux_out;
  sc_signal<regflit >	header;
  sc_signal<bool >		free[NPORT];

  // interface do Fila
  fila	*myQueue[NPORT];
    
  switch_control *mySwitchControl;
  
  void upd_header();
  void upd_dataout();
  void upd_dataack();
  void upd_tx();
  void upd_ackh();
  void upd_sgndataack();
  void upd_dataav();
  void upd_h();
  void upd_rx();
  void upd_clock_rx();
  void upd_sender();
  void upd_sgnackrx();
  void upd_sgn_credit_o();
  void upd_clock_tx();

	SC_HAS_PROCESS(router_cc);
	router_cc(sc_module_name name_, regaddress address_ = 0x00) :
	sc_module(name_), address(address_)
	{
		char temp[20];

		unsigned int i;
				
		for(i=0; i<NPORT; i++)
		{
			memset(temp, 0, sizeof(temp)); sprintf(temp,"fila%d",i);
			myQueue[i] = new fila(temp);
			myQueue[i]->clock(clock);
			myQueue[i]->reset_n(reset_n);
			myQueue[i]->data_in(data_in[i]);
			myQueue[i]->rx(rx[i]);
			myQueue[i]->credit_o(credit_o[i]);
			myQueue[i]->h(sgn_h[i]);
			myQueue[i]->ack_h(sgn_ack_h[i]);
			myQueue[i]->data_av(sgn_data_av[i]);
			myQueue[i]->data(data[i]);
			myQueue[i]->data_ack(sgn_data_ack[i]);
			myQueue[i]->sender(sgn_sender[i]);
		}

		mySwitchControl = new switch_control("novoswitchcontrol",address);
		mySwitchControl->clock(clock);
		mySwitchControl->reset(reset_n);
		for(i=0; i<NPORT; i++){
			mySwitchControl->data[i](data[i]);
			mySwitchControl->h[i](sgn_h[i]);
			mySwitchControl->ack_h[i](sgn_ack_h[i]);
			mySwitchControl->sender[i](sgn_sender[i]);
			mySwitchControl->free[i](free[i]);
		}
		mySwitchControl->mux_in(mux_in);
		mySwitchControl->mux_out(mux_out);

		SC_METHOD(upd_header);
		sensitive << incoming;
		sensitive << data[EAST];
		sensitive << data[WEST];
		sensitive << data[NORTH];
		sensitive << data[SOUTH];
		sensitive << data[LOCAL];

		SC_METHOD(upd_dataout);
		sensitive << free[EAST];
		sensitive << free[WEST];
		sensitive << free[NORTH];
		sensitive << free[SOUTH];
		sensitive << free[LOCAL];
		sensitive << data[EAST];
		sensitive << data[WEST];
		sensitive << data[NORTH];
		sensitive << data[SOUTH];
		sensitive << data[LOCAL];
		sensitive << mux_out;

		SC_METHOD(upd_dataack);
		sensitive << credit_i[EAST];
		sensitive << credit_i[WEST];
		sensitive << credit_i[NORTH];
		sensitive << credit_i[SOUTH];
		sensitive << credit_i[LOCAL];
		sensitive << mux_in;
		sensitive << sgn_data_av[EAST];
		sensitive << sgn_data_av[WEST];
		sensitive << sgn_data_av[NORTH];
		sensitive << sgn_data_av[SOUTH];
		sensitive << sgn_data_av[LOCAL];

		SC_METHOD(upd_tx);
		sensitive << free[EAST];
		sensitive << free[WEST];
		sensitive << free[NORTH];
		sensitive << free[SOUTH];
		sensitive << free[LOCAL];
		sensitive << sgn_data_av[EAST];
		sensitive << sgn_data_av[WEST];
		sensitive << sgn_data_av[NORTH];
		sensitive << sgn_data_av[SOUTH];
		sensitive << sgn_data_av[LOCAL];
		sensitive << mux_out;

		//SC_METHOD(upd_rx);
		//sensitive << rx;

		//SC_METHOD(upd_h);
		//sensitive << sgn_h[EAST];
		//sensitive << sgn_h[WEST];
		//sensitive << sgn_h[NORTH];
		//sensitive << sgn_h[SOUTH];
		//sensitive << sgn_h[LOCAL];

		//SC_METHOD(upd_ackh);
		//sensitive << ack_h;

		//SC_METHOD(upd_dataav);
		//sensitive << sgn_data_av[EAST];
		//sensitive << sgn_data_av[WEST];
		//sensitive << sgn_data_av[NORTH];
		//sensitive << sgn_data_av[SOUTH];
		//sensitive << sgn_data_av[LOCAL];

		//SC_METHOD(upd_sender);
		//sensitive << sgn_sender[EAST];
		//sensitive << sgn_sender[WEST];
		//sensitive << sgn_sender[NORTH];
		//sensitive << sgn_sender[SOUTH];
		//sensitive << sgn_sender[LOCAL];


		//SC_METHOD(upd_sgndataack);
		//sensitive << data_ack;
			
		//SC_METHOD(upd_sgn_credit_o);
		//sensitive << sgn_credit_o[EAST];
		//sensitive << sgn_credit_o[WEST];
		//sensitive << sgn_credit_o[NORTH];
		//sensitive << sgn_credit_o[SOUTH];
		//sensitive << sgn_credit_o[LOCAL];

		//SC_METHOD(upd_clock_tx);
		//sensitive << clock;

	}
	private:
		regaddress address;
};

#endif
