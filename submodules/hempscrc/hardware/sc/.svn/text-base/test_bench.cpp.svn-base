#include "test_bench.h"

#ifdef MTI_SYSTEMC
SC_MODULE_EXPORT(test_bench);
#endif

void test_bench::read_repository(){
	unsigned int address_local;
	
	address_local = (unsigned int)address.read()(25,0);
	
	if(read_req.read()==1 && read_req_ant.read()==1){
		
		if ( address_local < REPO_SIZE ){
			data_read.write(repository[address_local]);
		}
		data_valid.write(1);
	}
	else{
		if(read_req.read()==1 && read_req_ant==0){
			data_valid.write(0);
		}
	}
	read_req_ant.write(read_req.read());
}

void test_bench::debug_output(){
	sc_uint<32 >  l_data_out_debug;
	char c;
	static int line_length = 0;
	static bool str_end = false;
	ostringstream aux;
	int i;
	
	enum state {S0, S1};
	static state CS;
		
	busy_debug.write(0);
			
	if(write_enable_debug.read() == 1){
		//Reads the incoming string
		l_data_out_debug = data_out_debug.read();
		switch(CS){
			case S0:
				
				//Verifies if the string is from Echo()
				if(l_data_out_debug.range(7,0) == '$'){
					//write(file_line, line_type);
					str_out += '$';
					line_length = line_length + 1;
					CS = S1;
				
				//Writes the string to the file
				}
				else{
					str_end = false;
					
					for(i=0;i<4;i++){
						c = l_data_out_debug.range(31-i*8,24-i*8);
						//Writes a string in the line
						if(c != 10 && c != 0 && !(str_end)){
							str_out += c;
							line_length = line_length + 1;
						}
						//Detects the string end
						else if(c == 0){
							str_end = true;
						}
						//Line feed detected. Writes the line in the file
						else if(c == 10){
							str_out += c;
							line_length = 0;
						}
					}
				}
			break;
			//Receives from plasma the source processor, source task and writes them to the file
			case S1:
				str_out += ",";
				c = l_data_out_debug.range(7,0);
				aux<<(int)c;
				str_out += aux.str();
				line_length = line_length + 1;
				
				if(line_length == 3){
					str_out += ",";
					CS = S0;
				}
				else{
					CS = S1;
				}
			break;
		}
	}
}

void test_bench::log_gen(){
	sc_uint<32 >  l_data_log[N_PE];
	char c;
	static bool str_end = false;
	int i, j;
	
	for(j=0;j<N_PE;j++)
	{
		if(data_av[j].read() == 1)
		{
			l_data_log[j] = data_log[j].read();
			str_end = false;
			for(i=0;i<4;i++)
			{
				c = l_data_log[j].range(31-i*8,24-i*8);
				//Writes a string in the line
				if(c != 10 && c != 0 && !(str_end)){
					outstr[j] += c;
				}
				//Detects the string end
				else if(c == 0){
					str_end = true;
				}
				//Line feed detected. Writes the line in the file
				else if(c == 10){
					outstr[j] += c;
				}
			}
		}
	}
	
}

void test_bench::apps_dynamic_insertion(){
	
		int k = 0;
		int j = 0;
       
		if (reset.read() == 1) 
		{
			req_task.write(0);
		}
		else
		{
			for(j=0;j<NUMBER_OF_APPS;j++)
			{
				wait(appstime[j], SC_MS);
				repository[0] = repository[0] + dynamic_apps[j][0];
				k = 1;
				while (dynamic_apps[j][k]!=0xffffffff && k<11)
				{
					req_task.write(dynamic_apps[j][k] | 0x80000000);
					printf("Pediu a tarefa %d\n", dynamic_apps[j][k]);
					wait(ack_task.posedge_event());
					printf("Recebeu ack!\n");
					req_task.write(0);
					k++;
					wait(ack_task.negedge_event());
				}
			}
		}
}

void test_bench::ClockGenerator(){
	while(1){
		clock.write(0);
		wait (10, SC_NS);					//Allow signals to set
		clock.write(1);
		wait (10, SC_NS);					//Allow signals to set
	}
}
	
void test_bench::resetGenerator(){
	reset.write(1);
	wait (70, SC_NS);
	reset.write(0);
}

	
