#ifndef _logfilegen_h
#define _logfilegen_h
#include <systemc.h>
#include <iostream>
#include <string>
using namespace std;

SC_MODULE(logfilegen) {
	
	sc_in<bool >			data_av;
	sc_in<sc_uint<32> >		data_log;
	
	ofstream fp_out;//output file descriptor
	string str_out;//string to be printed in file
	
	void debug_output();
	
	SC_HAS_PROCESS(logfilegen);
	logfilegen(sc_module_name name_, char *filename_= "log.txt") :
    sc_module(name_), filename(filename_)
    {
		fp_out.open(filename, ios::out);
		if(!fp_out) {
			cout << "Cannot open " << filename << " file.\n";
		}
		
		SC_METHOD(debug_output);
		sensitive << data_av;
	}
	~logfilegen(){
		fp_out << str_out;
		cout<<str_out<<endl;
		fp_out.close();
	}
	private:
		char *filename;
};
#endif
