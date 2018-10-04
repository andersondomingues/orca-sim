#include "plasma_slave.h"

//#ifdef MTI_SYSTEMC
//SC_MODULE_EXPORT(plasma_slave);
//#endif

void plasma_slave::mem_mapped_registers(){
	
	sc_uint <32 > l_cpu_mem_address_reg = cpu_mem_address_reg.read();
	
	if(l_cpu_mem_address_reg.range(30,28 ) == 1){
		cpu_mem_data_read.write(data_read_reg.read());
	}
	else{
		switch(l_cpu_mem_address_reg){
			//case DEBUG:
			//	cpu_mem_data_read.write(data_read_debug.read());
			//break;
			case IRQ_MASK:
				cpu_mem_data_read.write(irq_mask_reg.read());
			break;
			case IRQ_STATUS_ADDR:
				cpu_mem_data_read.write(irq_status.read());
			break;
			case COUNTER:
				cpu_mem_data_read.write(time_slice.read());
			break;
			case NI_STATUS_READ:
				cpu_mem_data_read.write(ni_read_av.read());
			break;
			case NI_STATUS_SEND:
				cpu_mem_data_read.write(ni_send_av.read());
			break;
			case NI_READ:
				cpu_mem_data_read.write(ni_data_read.read());
			break;
			case NI_CONFIGURATION:
				cpu_mem_data_read.write(ni_config.read());
			break;
			case TICK_COUNTER_ADDR:
				cpu_mem_data_read.write(tick_counter.read());
			break;
			case DMA_ACTIVE:
				cpu_mem_data_read.write(dma_active_sig.read());
			break;
			case FIFO_AVAIL:
				cpu_mem_data_read.write(empty_out.read());
			default:
				cpu_mem_data_read.write(data_read_ram.read());
			break;
		}
	}
}


void plasma_slave::comb_assignments(){
	sc_uint<8 > l_irq_status;
	
	addr_a.write(cpu_mem_address.read()(31,2));
	addr_b.write(dma_mem_address.read()(31,2));
	
	
	write_enable_debug.write((((cpu_mem_address_reg.read() == DEBUG) && (write_enable.read() == 1))) ? 1  : 0 );
	
	
	data_av.write((((cpu_mem_address_reg.read() == DEBUG) && (write_enable.read() == 1))) ? 1  : 0 );
	data_log.write(cpu_mem_data_write_reg.read());
	
	data_out_debug.write(cpu_mem_data_write_reg.read());
	
	debug_write_busy.write(busy_debug.read());
	
	debug_busy.write(((((cpu_mem_address_reg.read() == DEBUG) && (write_enable.read() == 1)) && (busy_debug.read() == 1))) ? 1  : 0 );
		
	cpu_mem_pause.write(plasma_hold.read() | debug_busy.read() | mem_hold.read());
	irq.write((((irq_status.read() & irq_mask_reg.read()) != 0x00)) ? 1  : 0 );
	cpu_read_data.write((((cpu_mem_address_reg.read() == NI_READ) && (write_enable.read() == 0))) ? 1  : 0 );
	cpu_send_data.write((((cpu_mem_address_reg.read() == NI_WRITE) && (write_enable.read() == 1))) ? 1  : 0 );
	cpu_packet_ack.write((((cpu_mem_address_reg.read() == NI_ACK) && (write_enable.read() == 1))) ? 1  : 0 );
	cpu_packet_nack.write((((cpu_mem_address_reg.read() == NI_NACK) && (write_enable.read() == 1))) ? 1  : 0 );
	cpu_packet_end.write((((cpu_mem_address_reg.read() == NI_END) && (write_enable.read() == 1))) ? 1  : 0 );
	cpu_set_size.write((((cpu_mem_address_reg.read() == DMA_SIZE) && (write_enable.read() == 1))) ? 1  : 0 );
	cpu_set_address.write((((cpu_mem_address_reg.read() == DMA_ADDR) && (write_enable.read() == 1))) ? 1  : 0 );
	cpu_set_op.write((((cpu_mem_address_reg.read() == DMA_OP) && (write_enable.read() == 1))) ? 1  : 0 );
	cpu_start.write((((cpu_mem_address_reg.read() == START_DMA) && (write_enable.read() == 1))) ? 1  : 0 );
	cpu_ack.write((((cpu_mem_address_reg.read() == DMA_ACK) && (write_enable.read() == 1))) ? 1  : 0 );
	ni_send_data.write((dma_send_data.read() | cpu_send_data.read() ));
	ni_read_data.write((dma_read_data.read() | cpu_read_data.read() ));
	ni_data_write.write(((dma_send_data.read() == 1)) ? dma_data_write.read()  : cpu_mem_data_write_reg.read() );
	dma_data_read.write(((((cpu_set_op.read() == 1) || (cpu_set_size.read() == 1)) || (cpu_set_address.read() == 1))) ? cpu_mem_data_write_reg.read()  : ni_data_read.read() );
	//dma_mem_data_read.write(((dma_enable_internal_ram.read() == 1)) ? mem_data_read.read()  : data_read.read() );
	dma_mem_data_read.write(mem_data_read.read());
	//write_byte_enable.write(((cpu_enable_internal_ram.read() == 0)) ? cpu_mem_write_byte_enable_reg.read()  : ((dma_enable_internal_ram.read() == 0)) ? dma_mem_write_byte_enable.read()  : "0b0000" );
	
	/*if(cpu_enable_internal_ram.read() == 0){
		data_write.write(cpu_mem_data_write_reg.read());
	}
	else{
		if(dma_enable_internal_ram.read() == 0){
			data_write.write(dma_mem_data_write.read());
		}
		else{
			data_write.write(0);
		}
	}*/	
	
	write_enable.write(((cpu_mem_write_byte_enable_reg.read() != 0)) ? 1  : 0 );
	cpu_enable_internal_ram.write(((cpu_mem_address.read()(30,28 ) == 0)) ? 1  : 0 );
	dma_enable_internal_ram.write(((dma_mem_address.read()(30,28 ) == 0)) ? 1  : 0 );
	
	//irq_status              	<=  "00" & ni_intr & dma_intr & time_slice(14) & not time_slice(14) & not debug_write_busy & '0';-- debug_data_avail;                       
	//irq_status.write(((((((ni_intr.read()),dma_intr.read()),time_slice.read()[14 ]),!(time_slice.read()[14 ])),!(debug_write_busy.read())),0) );
	
	//irq_status.write(0)[7 ];
	//irq_status.write(0)[6 ];
	//irq_status.write(ni_intr.read())[5 ];
	//irq_status.write(dma_intr.read())[4 ];
	//irq_status.write(time_slice[14].read())[3 ];
	//irq_status.write(!(time_slice[14].read()))[2 ];
	//irq_status.write(!(debug_write_busy.read()))[1 ];
	//irq_status.write(0)[0 ];
	
	l_irq_status[7] = 0;
	l_irq_status[6] = 0;
	l_irq_status[5] = ni_intr.read();
	l_irq_status[4] = dma_intr.read();
	l_irq_status[3] = time_slice.read()[14 ];
	l_irq_status[2] = !(time_slice.read()[14 ]);
	l_irq_status[1] = !(debug_write_busy.read());
	l_irq_status[0] = 0;
	
	irq_status.write(l_irq_status);
}

void plasma_slave::reset_n_attr(){
	reset_n.write(!reset.read());
}

void plasma_slave::sequential_attr(){
	if (reset.read() == 1) {
		cpu_mem_address_reg.write(0);
		cpu_mem_data_write_reg.write(0);
		cpu_mem_write_byte_enable_reg.write(0);
		irq_mask_reg.write(0);
		time_slice.write(0);
		tick_counter.write(0);
  }
  else{
		if(cpu_mem_pause.read() == 0) {
			cpu_mem_address_reg.write(cpu_mem_address.read());
			
			cpu_mem_data_write_reg.write(cpu_mem_data_write.read());
			
			cpu_mem_write_byte_enable_reg.write(cpu_mem_write_byte_enable.read());
			
			if(cpu_mem_address_reg.read()==IRQ_MASK && write_enable.read()==1){
				irq_mask_reg.write(cpu_mem_data_write_reg.read()(7,0));
			}
			if(current_page.read() != 0){
				time_slice.write((time_slice.read() + 1) );
			}
		}
		if((cpu_mem_address_reg.read() == COUNTER) && (write_enable.read() == 1)){
			time_slice.write(cpu_mem_data_write_reg.read());
		}
		tick_counter.write((tick_counter.read() + 1) );
	}
}
