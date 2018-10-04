#include "Network_Interface.h"

//#ifdef MTI_SYSTEMC
//SC_MODULE_EXPORT(Network_Interface);
//#endif

void Network_Interface::config_update() {
	config.write(address_router); //vai sab !
}

void Network_Interface::send_av_update() {
	if(SS.read() == S3 || credit_i.read() == 0){
		send_av.write(0);
	}
	else{
		send_av.write(1);
	}
}

void Network_Interface::tx_update() {
	tx.write(tx_reg.read());
}

void Network_Interface::clock_tx_update() {
	clock_tx.write(clock.read());
}

void Network_Interface::send_process() {
	
	if (reset.read() == 1){ 
		tx_reg.write(0);
		SS.write(S0);
		data_out.write(0);
	}
	else{
		switch(SS.read()){
			
			// Waits the header flit
			case S0:
				if (send_data == 1){
					// Sends the header flit
					tx_reg.write(1);
					data_out.write(data_write.read()(TAM_FLIT-1,0));
					SS.write(S1);
				}
				else{
					SS.write(S0);
				}
			break;
			
			// Waits the payload size flit
			case S1:
				// Sends the multicast header and payload size flits
				if (credit_i.read() == 1 && send_data.read() == 1){
					tx_reg.write(1);
					data_out.write(data_write.read()(TAM_FLIT-1,0));
					payload_size.write(data_write.read()(TAM_FLIT-1,0));
					SS.write(S2);
				}
				else{
					if (credit_i.read() == 1) tx_reg.write(0);								
					else SS.write(S1);
				}
			break;	
			// Sends the high word flit
			case S2:							
				if (credit_i.read() == 1){
					// Verifies if there is a word to send
					if (send_data.read() == 1){
						data_out.write(data_write.read()(31,TAM_FLIT));
						low_word.write(data_write.read()(TAM_FLIT-1,0));
						tx_reg.write(1);
					}
					else{
						tx_reg.write(0);
					}
					
					// Decrement the payload size
					if (tx_reg.read() == 1) payload_size.write(payload_size.read() - 1);
																					
					// Decides the next state
					if (payload_size.read() == 0){
						tx_reg.write(0);									
						SS.write(S0);
					}	
					else{
						if (send_data.read() == 1) SS.write(S3);								
						else SS.write(S2);
					}								
				}	 
				// Waits for credit
				else SS.write(S2);
			break;
			// Sends the low word flit
			case S3:
				if (credit_i.read() == 1)
				{								
					// Sends the low word 
					data_out.write(low_word.read());
																			
					payload_size.write(payload_size.read() - 1);
													
					// Decides the next state
					if (payload_size.read() == 0) SS.write(S0);
					else SS.write(S2);
				}													
				// Waits for credit
				else SS.write(S3);
			break;	
			default: break;
		}
	}
}

//void Network_Interface::data_read_update() {
	//data_read.write(buffer_read.read()(31,0));
//}

void Network_Interface::credit_o_update() {
	credit_o.write(slot_available);
}

void Network_Interface::receive_process() {
	sc_uint<33 > l_buffer_write;
	
	// Receives data from NoC and stores in the Receive buffer
	if (reset.read() == 1){
		last.write(0);
		we.write(0);
		slot_available.write(1);
		header_stored.write(false);
		payload_size_readed.write(false);
		SR.write(SR0);
	}
	else{
		switch(SR.read()){
			// Waits the header flit
			case SR0:				
				// Receive the header flit 
				if (rx.read() == 1 && slot_available.read() == 1){
					l_buffer_write[31]=data_in.read()[15];
					l_buffer_write[30]=data_in.read()[14];
					l_buffer_write[29]=data_in.read()[13];
					l_buffer_write[28]=data_in.read()[12];
					l_buffer_write[27]=data_in.read()[11];
					l_buffer_write[26]=data_in.read()[10];
					l_buffer_write[25]=data_in.read()[ 9];
					l_buffer_write[24]=data_in.read()[ 8];
					l_buffer_write[23]=data_in.read()[ 7];
					l_buffer_write[22]=data_in.read()[ 6];
					l_buffer_write[21]=data_in.read()[ 5];
					l_buffer_write[20]=data_in.read()[ 4];
					l_buffer_write[19]=data_in.read()[ 3];
					l_buffer_write[18]=data_in.read()[ 2];
					l_buffer_write[17]=data_in.read()[ 1];
					l_buffer_write[16]=data_in.read()[ 0];
					
					buffer_write.write(l_buffer_write);
					SR.write(SR1);
				}
				else{
					SR.write(SR0);
				}
				
				if (we.read() == 1){
					we.write(0);
					last.write(last.read() + 1);
				}
			break;
			// Receives the low word flit
			case SR1:
				// Receives the flit
				if (rx.read() == 1 && slot_available == 1){
					
					l_buffer_write = buffer_write.read();
					
					l_buffer_write[15]=data_in.read()[15];
					l_buffer_write[14]=data_in.read()[14];
					l_buffer_write[13]=data_in.read()[13];
					l_buffer_write[12]=data_in.read()[12];
					l_buffer_write[11]=data_in.read()[11];
					l_buffer_write[10]=data_in.read()[10];
					l_buffer_write[ 9]=data_in.read()[ 9];
					l_buffer_write[ 8]=data_in.read()[ 8];
					l_buffer_write[ 7]=data_in.read()[ 7];
					l_buffer_write[ 6]=data_in.read()[ 6];
					l_buffer_write[ 5]=data_in.read()[ 5];
					l_buffer_write[ 4]=data_in.read()[ 4];
					l_buffer_write[ 3]=data_in.read()[ 3];
					l_buffer_write[ 2]=data_in.read()[ 2];
					l_buffer_write[ 1]=data_in.read()[ 1];
					l_buffer_write[ 0]=data_in.read()[ 0];
					
					//buffer_write.write(l_buffer_write);
					we.write(1);
					
					// Stores the payload size
					if (payload_size_readed.read()){
						payload_size_receive.write(payload_size_receive.read()-1);
					}
					else{
						payload_size_receive.write(data_in.read());
						payload_size_readed.write(true);
					}
					
					// Sets the last packet flit
					if (payload_size_receive.read() == 1){
						//l_buffer_write = buffer_write.read();
						l_buffer_write[32] = 1;
						buffer_write.write(l_buffer_write);
						payload_size_readed.write(false);
						SR.write(SR0);
					}
					else{
						//l_buffer_write = buffer_write.read();
						l_buffer_write[32] = 0;
						buffer_write.write(l_buffer_write);							
						SR.write(SR2);
					}
					//buffer_write.write(l_buffer_write);
				}
				else{
					SR.write(SR1);
				}
			break;	
			// Receives the high word flit
			case SR2 :																						
				// Stores the flit in the receive buffer
				if (we.read() == 1){
					we.write(0);
					last.write(last.read() + 1);								
				}
				
				// Receives the flit
				if (rx.read() == 1 && slot_available.read() == 1){
					l_buffer_write[31]=data_in.read()[15];
					l_buffer_write[30]=data_in.read()[14];
					l_buffer_write[29]=data_in.read()[13];
					l_buffer_write[28]=data_in.read()[12];
					l_buffer_write[27]=data_in.read()[11];
					l_buffer_write[26]=data_in.read()[10];
					l_buffer_write[25]=data_in.read()[ 9];
					l_buffer_write[24]=data_in.read()[ 8];
					l_buffer_write[23]=data_in.read()[ 7];
					l_buffer_write[22]=data_in.read()[ 6];
					l_buffer_write[21]=data_in.read()[ 5];
					l_buffer_write[20]=data_in.read()[ 4];
					l_buffer_write[19]=data_in.read()[ 3];
					l_buffer_write[18]=data_in.read()[ 2];
					l_buffer_write[17]=data_in.read()[ 1];
					l_buffer_write[16]=data_in.read()[ 0];
					
					buffer_write.write(l_buffer_write);
					
					payload_size_receive.write(payload_size_receive.read() - 1);
					SR.write(SR1);
				}
				else{
					SR.write(SR2);
				}
			break;
			default:
			break;
		}
	
		// Controls the available space in the receive buffer
		if (we.read() == 1 && ((first.read() == last.read()+2) || (first.read() == 0 && last.read() == 0xE) || (first.read() == 1 && last.read() == 0xF)) && read_data.read() == 0){
			slot_available.write(0);						
		}
		else{
			if (read_data.read() == 1){
				slot_available.write(1);
			}
		}
	}
}
//void Network_Interface::read_av_update() {
	//// Output registred
	//read_av.write(read_av_reg.read());
//}

void Network_Interface::r_buffer(){
	int i = first.read();
	buffer_read.write(buffer[i].read());
	data_read.write(buffer[i].read()(31,0));
}

void Network_Interface::w_buffer(){
	if(we.read()==1){
		int i = last.read();
		buffer[i].write(buffer_write.read());
	}
}


void Network_Interface::buffer_read_control() {
	//Controls the Plasma reading from the receive buffer
	if (reset.read() == 1){
		intr.write(0);
		hold.write(0);
		first.write(0);
		read_av.write(0);						
		SP.write(SR0);
	}
	else{
		switch(SP.read()){
			// Interrupts the Plasma processor
			case SR0:
				if (first.read() != last.read()){
					intr.write(1);
					SP.write(SR1);
				}
				else{
					SP.write(SR0);
				}
			break;	
			// Controls the receive buffer reading
			case SR1:
				if (read_data.read() == 1){
					intr.write(0);
					first.write(first.read() + 1);
					
					if (buffer_read.read()[32 ] == 1){
						SP.write(SR0);
					}
					else{
						SP.write(SR1);
					}
				}
				else{
					SP.write(SR1);
				}
			break;										
			default:
			break;
		}
		
		// Controls the read availability to Plasma processor
		if (we.read() == 1){
			read_av.write(1);
		}
		else{	
			if (read_data.read() == 1 && ((first.read() + 1) == last.read() || (first.read()==0xF && last.read()==0))){
				read_av.write(0);
			}
			else{
				read_av.write(read_av.read());
			}
		}
	}
}
