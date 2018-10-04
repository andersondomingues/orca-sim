#include "access_repository.h"

//#ifdef MTI_SYSTEMC
//SC_MODULE_EXPORT(access_repository);
//#endif

void access_repository::fsm_sequ_assign() {
	
	if (reset.read() == 1){ 
		ea.write(wait_addr);
		mem_hold.write(0);
		read_req.write(0);
		address.write(0);
		data_read_reg.write(0);
	}
	else {
		ea.write(pe);
		if(data_valid.read()==1) {
			data_read_reg.write(data_read.read());
		}
	}
	switch(ea.read()){
		
		case wait_addr:
			if (cpu_mem_address.read()(30,28) == 1){
				mem_hold.write(1);
			}
			else {
				if (mem_ddr_access.read() == 1){
					read_req.write(1);
					address.write(dma_mem_addr_ddr.read()(31,2));
					mem_hold.write(0);
				}
			}
		break;
		
		case set_req:
			address.write(cpu_mem_address_reg.read()(31,2));
			read_req.write(1);
		break;	
		
		case wait_data:							
			address.write(cpu_mem_address_reg.read()(31,2));
			if(data_valid.read() == 1){
				mem_hold.write(0);
				read_req.write(0);
			}
			else read_req.write(1);
		break;
		
		case dma_access:
			read_req.write(dma_mem_ddr_read_req);
			address.write(dma_mem_addr_ddr.read()(31,2));
		break;
		
		case set_done:
			read_req.write(0);
		break;	
		
		default: break;
	}
}

void access_repository::fsm_comb_assign(){
	
	switch(ea.read()){
		
		case wait_addr:
			if(cpu_mem_address.read()(30,28) == 1){
				pe.write(set_req);
			}
			else{
				if (mem_ddr_access.read() == 1){
					pe.write(dma_access);
				}
				else{
					pe.write(wait_addr);
				}
			}
		break;
		
		case set_req:
			pe.write(wait_data);
		break;	
		
		case wait_data:							
			if(data_valid.read() == 1){
				pe.write(set_done);
			}
			else{
				pe.write(wait_data);
			}
		break;
		
		case dma_access:
			if(mem_ddr_access.read() == 1){
				pe.write(dma_access);
			}
			else{
				pe.write(set_done);
			};
		break;
		
		case set_done:
			pe.write(wait_addr);
		break;	
		
		default: break;
	}
}
