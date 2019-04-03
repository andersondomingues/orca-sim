#include <orca-lib.h>
#include <orca-remote.h>

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <cstring>

#define SERVICE_PORTNUM 5000
#define SERVICE_CHANNEL 5000
#define SERVICE_UDPADDR "127.0.0.1"
#define SERVICE_UDPPORT 9999

/* displays help info */
void display_help(){
	std::cout <<
		"Usage: orca-remote-control -CMD [p1 [p2 [p3 ...]]]" << std::endl <<
		"  -map p1 p2: maps task p2 (string) to core p1 (integer)." << std::endl <<
		"  -migrate p1 p2 p3: migrate task p3 (string) from core p2 (integer) to core p1 (integer)"
		<< std::endl;
}

/* remotely migrate some task*/
void f_migrate(int argc, char** argv){
	argc = 1;
	argv[argc][argc] = 'c';
}

/* remotely maps some task */
void f_map(int argc, char** argv){
	
	//0 - exe name, 1 - core number, 2 - task name
	if(argc == 3){
		char* task_name = argv[2];
		int32_t core_num, task_name_len;
		
		core_num = atoi(argv[1]);
		task_name_len = strlen(task_name);	

		int8_t buf[500];
		buf[0] = core_num;
		buf[1] = task_name_len;
		strcpy((char*)&buf[1], task_name);
		
		dump((char*)buf, 0, task_name_len * 2);
		hf_send(core_num, SERVICE_PORTNUM, buf, task_name_len * 2, SERVICE_CHANNEL, SERVICE_UDPADDR, SERVICE_UDPPORT);
		
	}else{
		display_help();	
	}
}

/* entry-point */
int main(int argc, char** argv){ 

	//display help when no parameter informed 
	if(argc == 1){
		display_help();			
	}

	//Call specific function 	
	else if(strcmp(argv[1], "-map") == 0)
		f_map(argc, argv); 
	else if(strcmp(argv[1], "-migrate") == 0)
		f_migrate(argc, argv);
	else
		display_help();
		
	return 0;
}
