#include "dma_master.h"

//#ifdef MTI_SYSTEMC
//SC_MODULE_EXPORT(dma_master);
//#endif

void dma_master::comb_assign(){
	if(operation == 1){
		mem_address.write(address.read()-4);
	}
	else{
		mem_address.write(address.read());
	}
	
	if(EA.read() == SCopyToMem && read_av.read() == 1){
		read_data.write(1);
	}
	else{
		read_data.write(0);
	}
	
	mem_addr_ddr.write(address.read());
}
	
void dma_master::dma_fsm(){

	if(reset.read() == 1){
		mem_data_read_reg.write(0);
		data_valid_reg.write(0);	
		
		address.write(0);
		mem_data_write.write(0);
		size.write(0);
		
		intr.write(0);
		data_write_reg.write(0);
		operation.write(0);
		mem_byte_we.write(0);
		data_reg.write(0);
		read_data_reg.write(0);
		EA.write(SWait);
		active.write(0);
		
		data_write.write(0);
		send_data.write(0);
		mem_ddr_read_req.write(0);
		mem_ddr_access.write(0);
	}
	else{
		mem_data_read_reg.write(mem_data_read.read());
		data_valid_reg.write(data_valid.read());
		
		switch(EA.read()){
			case SWait:
				if(intr_ack.read() == 1){
					intr.write(0);
					EA.write(SWait);
				}
				else{
					if(set_size.read() == 1){
						size.write(data_read.read()(15,0));
						EA.write(SWait);
					}
					else{
						if(set_op.read() == 1){
							operation.write(data_read.read()(0,0));
							EA.write(SWait);
						}
						else{
							if(set_address.read() == 1){
								address.write(data_read.read());
								EA.write(SWait);
							}
							else{
								if(start.read() == 1){
									active.write(1);
									if(operation.read() == 0){
										if(address.read()(30,28) == 1){
											EA.write(SWaitMem0);
											size.write(size.read()-1);//decrementa a escrita da primeira palavra
											mem_ddr_access.write(1);
											mem_ddr_read_req.write(1);
										}
										else{
											EA.write(SCopyFromRam);
											mem_ddr_read_req.write(0);
										}
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
			case SCopyFromRam0:
				if(send_av.read() == 1){
					address.write(address.read()+4);
					size.write(size.read()-1);
					EA.write(SCopyFromRam);
				}
				else{
					EA.write(SCopyFromRam0);
				}
			break;
			
			case SCopyFromRam:
				if(send_av.read() == 1){
					EA.write(SCopyFromRamWait);
					address.write(address.read()+4);
					size.write(size.read()-1);
					send_data.write(1);
					data_write.write(mem_data_read_reg.read());
				}
				else{
					send_data.write(0);
					data_write.write(mem_data_read_reg.read());
					EA.write(SCopyFromRam);
				}
			break;
			
			case SCopyFromRamWait:
				if(size.read() == 0){
					EA.write(SEnd);
					send_data.write(0);
				}
				else{
					EA.write(SCopyFromRamWait1);
					data_write.write(mem_data_read_reg.read());
					send_data.write(0);
				}
			break;
			
			case SCopyFromRamWait1:
				EA.write(SCopyFromRam);
			break;
			
			case SWaitMem0: //espera baixar data_valid
				if(data_valid_reg.read() == 0){
					EA.write(SWaitMem1);
				}
				else{
					EA.write(SWaitMem0);
				}
			break;
			
			case SWaitMem1:
				if(data_valid_reg.read() == 1){
					EA.write(SCopyFromMem);
					mem_ddr_read_req.write(0);
				}
				else{
					EA.write(SWaitMem1);
				}
				send_data.write(0);
			break;
			
			case SCopyFromMem:
				if((send_av.read() == 1) && (data_valid_reg.read() == 1)){
					address.write(address.read()+4);
					size.write(size.read()-1);
					if(size.read() == 0){
						EA.write(SEnd);
						mem_ddr_read_req.write(0);
					}
					else{
						EA.write(SWaitMem1);
						mem_ddr_read_req.write(1);
					}
					send_data.write(1);
					data_write.write(mem_data_read_reg.read());
				}
				else{
					send_data.write(0);
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
				send_data.write(0);
				mem_ddr_access.write(0);
			break;
			
			default:
			break;
		}
	}
}
