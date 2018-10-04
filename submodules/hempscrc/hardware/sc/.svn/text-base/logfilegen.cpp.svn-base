#include "logfilegen.h"

void logfilegen::debug_output(){
	sc_uint<32 >  l_data_log;
	char c;
	static bool str_end = false;
	int i;
	
	l_data_log = data_log.read();
	
	if(data_av.read() == 1)
	{
		str_end = false;
		for(i=0;i<4;i++){
			c = l_data_log.range(31-i*8,24-i*8);
			//Writes a string in the line
			if(c != 10 && c != 0 && !(str_end)){
				str_out += c;
			}
			//Detects the string end
			else if(c == 0){
				str_end = true;
			}
			//Line feed detected. Writes the line in the file
			else if(c == 10){
				str_out += c;
			}
		}
	}
		//cout << "fdp " << filename << " entrou" << endl;
		//Reads the incoming string
		//l_data_log = data_log.read();
		//			str_end = false;
		//for(i=0;i<4;i++){
			//c = l_data_log.range(31,24);
			//aux<<(int)c;
			//str_out += aux.str();
			//c = l_data_log.range(23,16);
			//aux<<(int)c;
			//str_out += aux.str();
			//c = l_data_log.range(15,8);
			//aux<<(int)c;
			//str_out += aux.str();
			//c = l_data_log.range(7,0);
			//aux<<(int)c;
			//str_out += aux.str();
			//if(c != 10 && c != 0 && !(str_end)){
				//str_out += c;
				//line_length = line_length + 1;
			//}
			//else if(c == 0){
				//str_end = true;
			//}
			//else if(c == 10){
				//str_out += c;
				//line_length = 0;
			//}
		//}
	//}
	//else{
	//	cout << "fdp " << filename << " nao gosta de mim!" << endl;
	//}
}
