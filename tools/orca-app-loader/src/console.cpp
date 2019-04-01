#include "udp_client_server.h"
#include "libhf.h"

#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <fstream>

//return true if file exists
bool file_exists(std::string path){
	std::ifstream f(path.c_str());
	return f.good();
}

//return the size (in bytes of some file)
std::ifstream::pos_type file_size(std::string path)
{
    std::ifstream in(path.c_str(), std::ifstream::ate | std::ifstream::binary);
    return in.tellg(); 
}


void load_bin(int8_t* buffer, std::string bin_path, uint32_t bin_size){

    //TODO: not sure it is the best performatic way
	std::ifstream f(bin_path, std::ios::binary | std::ios::in | std::ios::out);
	
	if(f.is_open()){
		f.read((char*)buffer, bin_size);
		f.close();
	}else{
	    //TODO: surround with try-catch instead of printing
		std::cout << "error: unable to load binary file." << std::endl;
		exit(0);
	}
}

int main(int argc, char** argv){ 

	//argv[0] = "orca-app-loader.exe"
	//argv[1] = core number
	//argv[2] = task file (path to binary)
	//argv[3] = task name (name will be used by the kernel)
	//argv[4] = server addr
	//argv[5] = server port

	if(argc != 6){
		std::cout <<
			"Usage: " << argv[0] << " core binpath name ipaddr ipport " << std::endl << std::endl <<
			"All parameters must be informed." << std::endl <<
			"Example: " << argv[0] << " 2 ./bin/image.bin image 127.0.0.1 9999" << std::endl << std::endl <<
			"Description of parameters:" << std::endl <<
			"     core:   Number of the tile to which the task would be loaded." << std::endl <<
			"  binpath:   Path to the file containing task binary core." << std::endl <<
			"     name:   A name for the task. Must be unique per core." << std::endl <<
			"   ipaddr:   IP address of the platform gateway." << std::endl <<
			"   ipport:   IP port of the platform gateway." << std::endl;
		return 0;
	}

	//parse core number
	uint32_t core_num = atoi(argv[1]);
	
	if(core_num == 0){
		std::cout << "error: core num cannot be zero." << std::endl;
		exit(0); 
	}

	//check for file
	std::string bin_path = std::string(argv[2]);
	if(!file_exists(bin_path)){
		std::cout << "error: could not open binary file." << std::endl;
		exit(0);	
	}
	
	//parse task name
	std::string task_name = std::string(argv[3]);
	std::cout << "Loading task \"" << task_name << "\" into core #" << core_num
		<< ", from [file:/" <<  bin_path << "]." << std::endl;

	//print binary size
	uint32_t bin_size = file_size(bin_path);
	std::cout << "Read " << bin_size << " bytes from binary file." << std::endl;

	//load file contents
	//buffer must be greater than file size due a simplification
	//made in the copy algorithm
	//TODO: make a better copy algorithm (see libhf.c)
	int8_t* buffer;
	buffer = new int8_t[bin_size * 2];
	
	//TODO: task name must be sent 
	load_bin(buffer, bin_path, bin_size);
	
	std::string server_addr = std::string(argv[4]);
	int server_port = atoi(argv[5]);	
	
	std::cout << "server running at " << server_addr << ":" << server_port << std::endl;
	
	//send via api
	hf_send(core_num, LOADER_SERVICE_PORT, buffer, bin_size, LOADER_SERVICE_CHANNEL, server_addr, server_port);
}
