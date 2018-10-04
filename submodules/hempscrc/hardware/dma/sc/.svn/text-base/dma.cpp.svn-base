#include "dma.h"

//#ifdef MTI_SYSTEMC
//SC_MODULE_EXPORT(dma);
//#endif

void dma::comb_assign(){
	if(operation == 1){
		mem_address.write(address.read()-4);
	}
	else{
		mem_address.write(address.read());
	}
	
	if(read_data_reg.read() == 1){
		data_write.write(data_reg.read());
	}
	else{
		data_write.write(mem_data_read.read());
	}
	
	if(EA.read() == SCopyToMem && read_av.read() == 1){
		read_data.write(1);
	}
	else{
		read_data.write(0);
	}
	
	if(EA.read() == SCopyFromMem && send_av.read() == 1){
		send_data.write(1);
	}
	else{
		send_data.write(0);
	}
}

void dma::dma_fsm(){
	if(reset.read() == 1){
        	address.write(0);
         	mem_data_write.write(0);
         	size.write(0);
         
         	intr.write(0);
         	operation.write(0);
         	mem_byte_we.write(0000);
         	data_reg.write(0);
         	read_data_reg.write(0);
         	active.write(0);
         	EA.write(SWait);
	}
	else{
		switch(EA.read()){
			case SWait:
				if(intr_ack.read() == 1 ){
					intr.write(0);
					EA.write(SWait);
				}
				else{
					if(set_size.read() == 1 ){
						size.write(data_read.read()(15,0));
						EA.write(SWait);
					}
					else{
						if(set_op.read() == 1 ){
							operation.write(data_read.read()(0,0));
							EA.write(SWait);
						}
						else{
							if(set_address.read() == 1 ){
								address.write(data_read.read());
								EA.write(SWait);
							}
							else{
								if(start.read() == 1 ){
									active.write(1);
									if(operation.read() == 0){
										EA.write(SCopyFromMem0);
									}
									else{
										EA.write(SCopyToMem);
									}
								}
							}
						}
					}
				}
			break;
			//Ensures that the first word will not be sent 2 times.
			case SCopyFromMem0:
				if(send_av.read() == 1){	
					address.write(address.read()+4);
					size.write(size.read()-1);
					EA.write(SCopyFromMem);
				}
				else{
					EA.write(SCopyFromMem0);
				}
			break;
			case SCopyFromMem:
				if(send_av.read() == 1){            	
					read_data_reg.write(0);
					address.write(address.read()+4);
					size.write(size.read()-1);
					if(size.read() == 0){
						EA.write(SEnd);
					}
					else{
						EA.write(SCopyFromMem);
					}
				}
				else{
					if(read_data_reg.read() == 0){//nao grava no reg pq ele deve ser lido na sequela
						data_reg.write(mem_data_read.read());
					}
					read_data_reg.write(1);
					EA.write(SCopyFromMem);
				}
			break;
			case SCopyToMem:
				if(read_av.read() == 1){
					mem_data_write.write(data_read.read());
					mem_byte_we.write(0xF);
					address.write(address.read()+4);
					size.write(size.read()-1);
					if(size.read() == 1){//The last word is sent in the Send state
						EA.write(SEnd);
					}
					else{
						EA.write(SCopyToMem);
					}
				}
				else{
				   mem_byte_we.write(0);
				}
			break;
			case SEnd:
				mem_byte_we.write(0);
				intr.write(1);
				active.write(0);
				EA.write(SWait);
			break;
		}
	}
}
