#include <systemc.h>

#include <HeMPS_PKG.h>
//#include "logfilegen.h"
#include "../plasma/sc/plasma_master.h"
#include "../plasma/sc/plasma_slave.h"

#define BL 0
#define BC 1
#define BR 2
#define CL 3
#define CC 4
#define CRX 5
#define TL 6
#define TC 7
#define TR 8



SC_MODULE(hemps) {
	
	sc_in< bool >			clock;
	sc_in< bool >			reset;

	//Tasks repository interface
	sc_out<bool >			read_req;
	sc_out<sc_uint<30> >	mem_addr;
	sc_in<sc_uint<32> >		data_read;
	sc_in<bool >			data_valid;
	sc_out< sc_uint<32> >	data_write;
	sc_out< sc_uint<4> >	write_byte_enable;
	
	//Debug interface
	sc_out<bool >			write_enable_debug;
	sc_out<sc_uint<32> >	data_out_debug;
	sc_in<bool >			busy_debug;
	
	//Dynamic Insertion of Applications
	sc_out<bool >			ack_task;
	sc_in<sc_uint<32> >		req_task;
	
	sc_out<bool >			data_av[N_PE];
	sc_out<sc_uint<32> >	data_log[N_PE];
	
		
	// NoC Interface
	sc_signal<bool >		clock_tx[N_PE][NPORT-1];
	sc_signal<bool >		tx[N_PE][NPORT-1];
	sc_signal<regflit >		data_out[N_PE][NPORT-1];
	sc_signal<bool >		credit_i[N_PE][NPORT-1];
	
	sc_signal<bool >		clock_rx[N_PE][NPORT-1];
	sc_signal<bool > 		rx[N_PE][NPORT-1];
	sc_signal<regflit >		data_in[N_PE][NPORT-1];
	sc_signal<bool >		credit_o[N_PE][NPORT-1];
		
	plasma_master *master;//only one master
	plasma_slave  *slave[N_PE];//others PEs are slaves
	
	//logfilegen	  				*log[N_PE];
	//sc_signal<bool >			data_av[N_PE];
	//sc_signal<sc_uint<32> >		data_log[N_PE];
	sc_signal<bool >			busy_debug_slave[N_PE];
	
	sc_signal<sc_uint<4> > 	pos[N_PE];
	
	int i,j;


	int RouterPosition(int router);
	regaddress RouterAddress(int router);
 	void pes_interconnection();
 	void log_gen();
	
	char temp[20];
	char logfile[20];
	SC_CTOR(hemps){
		for(j=0;j<N_PE;j++){
			//printf("creating %d - %d\n",j,(int)RouterPosition(j));
			printf("creating PE:%dX%d\n",j%N_PE_Y,j/N_PE_Y);
		}
		for(j=0;j<N_PE;j++){
			if(j==MASTER){
				memset(temp, 0, sizeof(temp)); sprintf(temp,"master%d",j);
				master = new plasma_master(temp,RouterAddress(j));
				master->clock(clock);
				master->reset(reset);
				master->read_req(read_req);
				master->address(mem_addr);
				master->data_read(data_read);
				master->data_valid(data_valid);
				master->data_write(data_write);
				master->write_byte_enable(write_byte_enable);
				master->write_enable_debug(write_enable_debug);
				master->data_out_debug(data_out_debug);
				master->busy_debug(busy_debug);
				master->ack_task(ack_task);
				master->req_task(req_task);
				for(i=0;i<NPORT-1;i++){
					master->clock_tx	[i]	(clock_tx	[j][i]);
					master->tx			[i]	(tx			[j][i]);
					master->data_out	[i]	(data_out	[j][i]);
					master->credit_i	[i]	(credit_i	[j][i]);
					master->clock_rx	[i]	(clock_rx	[j][i]);
					master->data_in		[i]	(data_in	[j][i]);
					master->rx			[i]	(rx			[j][i]);
					master->credit_o	[i]	(credit_o	[j][i]);
				}
			}
			else{
				memset(temp, 0, sizeof(temp)); sprintf(temp,"slave%d",j);
				memset(logfile, 0, sizeof(logfile)); sprintf(logfile,"log%d.txt",j);
				slave[j] = new plasma_slave(temp,RouterAddress(j),logfile);
				slave[j]->clock(clock); 
				slave[j]->reset(reset);
				slave[j]->write_enable_debug(data_av[j]);
				slave[j]->data_out_debug(data_log[j]);
				slave[j]->busy_debug(busy_debug_slave[j]);
				for(i=0;i<NPORT-1;i++){
					slave[j]->clock_tx	[i]	(clock_tx	[j][i]);
					slave[j]->tx		[i]	(tx			[j][i]);
					slave[j]->data_out	[i]	(data_out	[j][i]);
					slave[j]->credit_i	[i]	(credit_i	[j][i]);
					slave[j]->clock_rx	[i]	(clock_rx	[j][i]);
					slave[j]->data_in	[i]	(data_in	[j][i]);
					slave[j]->rx		[i]	(rx			[j][i]);
					slave[j]->credit_o	[i]	(credit_o	[j][i]);
				}
				/*log = new logfilegen(logfile, logfile);
				log->data_av(data_av[j]);
				log->data_log(data_log[j]);*/
			}
		}
	 		SC_METHOD(pes_interconnection);
			for(j=0;j<N_PE;j++){
				for(i=0;i<NPORT-1;i++){
					sensitive << clock_tx	[j][i];
					sensitive << tx			[j][i];
					sensitive << data_out	[j][i];
					sensitive << credit_i	[j][i];
					sensitive << clock_rx	[j][i];
					sensitive << data_in	[j][i];
					sensitive << rx			[j][i];
					sensitive << credit_o	[j][i];
				}
			}
				
	}
};

