#include "router_cc.h"

//#ifdef MTI_SYSTEMC
//SC_MODULE_EXPORT(router_cc);
//#endif

void router_cc::upd_header(){
         if(incoming.read()==EAST) header.write(data[EAST].read());
    else if(incoming.read()==WEST) header.write(data[WEST].read());
    else if(incoming.read()==NORTH) header.write(data[NORTH].read());
    else if(incoming.read()==SOUTH) header.write(data[SOUTH].read());
    else if(incoming.read()==LOCAL) header.write(data[LOCAL].read());
}

void router_cc::upd_dataout(){
	reg_mux localmux_out;
	localmux_out = mux_out.read();

	for (int i=0; i<NPORT; i++){
		sc_uint<3> j=localmux_out.range(i*3+2,i*3);
		if (i==j){
			data_out[i].write(0);
		}
		else{
			if (free[i].read()==0){
				data_out[i].write(data[j]);
			}
		}
	}
}


//void router_cc::received_flit(){
    //for (int i=0; i<NPORT; i++){
        //if ((tx[i].read() == 1) &&(credit_in[i].read() == 1)){
            //counter[i]++;
        //}
    //}
//}

void router_cc::upd_dataack(){
	reg_mux local_mux_in;
	

	local_mux_in=mux_in.read();

	for (int i=0; i<NPORT; i++){
		int j=local_mux_in.range(i*3+2,i*3);
		if (i==j){
			sgn_data_ack[i].write(0);
		}
		else{
			if (sgn_data_av[i].read()==1){
				sgn_data_ack[i].write(credit_i[j]);
			}
			else{
				sgn_data_ack[i].write(0);
			}
		}
	}
	
}


void router_cc::upd_tx(){
	reg_mux local_mux_out=mux_out.read();

	for (int i=0; i<NPORT; i++){
		int j=local_mux_out.range(i*3+2,i*3);
		if (i==j){
			tx[i].write(0);
		}
		else{
			if (free[i].read()==0){
				tx[i].write(sgn_data_av[j]);
			}
		}
	}
}


