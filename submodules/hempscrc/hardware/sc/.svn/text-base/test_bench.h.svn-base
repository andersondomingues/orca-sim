#include <systemc.h>
#include <iostream>
#include <string>

using namespace std;

#include "hemps.h"
#include <repository.h>
#include <dynamic_apps.h>

SC_MODULE(test_bench) {
	
	sc_signal< bool >	clock;
	sc_signal< bool >	reset;
		
	void read_repository();
	void ClockGenerator();
	void resetGenerator();
	void debug_output();
	void log_gen();
	void apps_dynamic_insertion();
	
	hemps *MPSoC;
	
	//Tasks repository interface
	sc_signal<sc_uint<30> >		address;
	sc_signal<bool >			read_req;
	sc_signal<bool >			read_req_ant;
	sc_signal<sc_uint<32> >		data_write;
	sc_signal<sc_uint<32> > 	data_read;
	sc_signal< sc_uint<4> >		write_byte_enable;
	sc_signal<bool >			data_valid;
	
	//Debug interface
	sc_signal<bool >			write_enable_debug;
	sc_signal<sc_uint<32> >		data_out_debug;
	sc_signal<bool >			busy_debug;
	
	//Dynamic Insertion of Applications
	sc_signal<bool > 			ack_task;
    sc_signal<sc_uint<32> >     req_task;
    
    sc_signal <	bool> data_av[N_PE];
	sc_signal < sc_uint <32 > > data_log[N_PE];
    
    ofstream fp_out;//output file descriptor
    ofstream filelog[N_PE];//output file descriptor
	
	string str_out;//string to be printed in file
	string outstr[N_PE];//string to be printed in file
	
	char logfile[20];
	
	SC_HAS_PROCESS(test_bench);
	test_bench(sc_module_name name_, char *filename_= "output_master.txt") :
    sc_module(name_), filename(filename_)
    {
	//SC_CTOR(test_bench){
		fp_out.open(filename, ios::out);
		if(!fp_out) {
			cout << "Cannot open " << filename << " file.\n";
		}
		
		for(int j=0;j<N_PE;j++)
		{
			memset(logfile, 0, sizeof(logfile)); sprintf(logfile,"log/log%d.txt",j);
			filelog[j].open(logfile, ios::out);
			if(!filelog[j]) {
				cout << "Cannot open " << logfile << " file.\n";
			}
		}
		
		MPSoC = new hemps("hemps");
		MPSoC->clock(clock);
		MPSoC->reset(reset);
		MPSoC->mem_addr(address);
		MPSoC->read_req(read_req);
		MPSoC->data_write(data_write);
		MPSoC->data_read(data_read);
		MPSoC->write_byte_enable(write_byte_enable);
		MPSoC->data_valid(data_valid);
		MPSoC->write_enable_debug(write_enable_debug);
		MPSoC->data_out_debug(data_out_debug);
		MPSoC->busy_debug(busy_debug);
		MPSoC->ack_task(ack_task);
		MPSoC->req_task(req_task);
		for(int j=0;j<N_PE;j++){
			MPSoC->data_av[j](data_av[j]);
			MPSoC->data_log[j](data_log[j]);
		}
		
		SC_METHOD(read_repository);
		sensitive << clock;
		
		SC_THREAD(apps_dynamic_insertion);
		sensitive << reset;
		
		SC_THREAD(ClockGenerator);

		SC_THREAD(resetGenerator);
		
		SC_METHOD(debug_output);
		sensitive << write_enable_debug;
		
		SC_METHOD(log_gen);
		for(int j=0;j<N_PE;j++) sensitive << data_av[j];
		
	}
	~test_bench(){
		for(int j=0;j<N_PE;j++)
		{
			filelog[j] <<outstr[j];
			filelog[j].close();
		}
		fp_out << str_out;
		fp_out.close();
	}
	private:
		char *filename;
	

};


#ifndef MTI_SYSTEMC

int sc_main(int argc, char *argv[]){
	int time_to_run=0;
	int i;
	char *filename = "output_master.txt";
	if(argc<3){
		cout << "Sintax: " << argv[0] << " -c <milisecons to execute> [-o <output filename>]" << endl;
		exit(EXIT_FAILURE);
	}
	
	InitializeAppsInfo();
		
	for (i = 1; i < argc; i++){/* Check for a switch (leading "-"). */
		if (argv[i][0] == '-') {/* Use the next character to decide what to do. */
			switch (argv[i][1]){
				case 'c':
					time_to_run = atoi(argv[++i]);
				break;
				case 'o':
					filename = argv[++i];
					cout << filename << endl;
				break;
				default:
					cout << "Sintax: " << argv[0] << "-c <milisecons to execute> [-o <output name file>]" << endl;
					exit(EXIT_FAILURE);
				break;
			}
		}
	}
	
	
	test_bench tb("testbench",filename);
	sc_start(time_to_run,SC_MS);
  	return 0;
}
#endif
