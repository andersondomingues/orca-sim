#include <iostream>

//write 
void add_noc_header(char* buffer){

	//add noc headers 
	buffer[0] = 0x11;  //(1,1) is core #5
	buffer[1] = 0x00; 

	buffer[2] = 0x3e; 
	buffer[3] = 0x00;  //length flit: 0x3e = 62 flits

	buffer[4] = 0x00;  //payload
	buffer[5] = 0x05;  //target_cpu (5)

	buffer[6] = 0xe8;	//src_port (5000)
	buffer[7] = 0x00;  //src_cpu (0,0)
	
	buffer[8] = 0x88;  //msg_size
	buffer[9] = 0x13;  //0x1388 = 5000 dec
	
	buffer[10] = 0x64;
	buffer[11] = 0x00;
	
	buffer[12] = 0x01;
	buffer[16] = 0x20;
}

//shorthand print
void dump(char* _mem, uint32_t base, uint32_t length){
	uint32_t k, l;
	
	//mask is necessary to correct a bug(?) when printing
	//negative hexas.
	uint32_t mask = 0x000000FF; 
	int8_t ch;
	
	//uint32_t* memptr = (uint32_t*)_mem;
	//uint32_t  len = _length / 4;
	for(k = 0; k < length; k += 16){
		printf("\n%08x ", base + k);
		for(l = 0; l < 16; l++){
			printf("%02x ", _mem[k + l] & mask );
			if (l == 7) putchar(' ');
		}
		printf(" |");
		for(l = 0; l < 16; l++){
			ch = _mem[k + l];
			if ((ch >= 32) && (ch <= 126))
				putchar(ch);
			else
				putchar('.');
		}
		putchar('|');
	}
}
