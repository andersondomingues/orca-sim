#include <RspServer.h>

#define RSP_DEBUG 1

RspServer::RspServer(std::string ipaddr, uint32_t udpport){
    
    _udpport = udpport;
    _ipaddr  = ipaddr;

    //create udp server
    _server = new UdpAsyncServer(udpport);
    
    //create a new udp socket with the informed parameters

    //register OnReceive as the listener

    //start listening
}


//remove udp server instance
RspServer::~RspServer(){
    delete _server;
}

//checksum
uint8_t RspServer::Checksum(char* buffer, int length)
{
    uint8_t checksum = 0;

	while (length--) 
		checksum += *buffer++;
	
	return checksum;
}

int RspServer::Respond(std::string data){
    

    //packet response should be "$data#checksum"
    //copy the $ charater to the output
    _output_buffer[0] = '$';

    //copy data into output buffer
    memcpy(&_output_buffer[1], data.c_str(), data.length());

    //add separator to the ourput
    _output_buffer[data.length() + 2] = '#';

    //calculate checksum
    uint8_t checksum = this->Checksum(_output_buffer, data.length());
    sprintf((char*)&_output_buffer[data.length() +3], "%02X", checksum);

    //append checksum to the data
    memcpy(&_output_buffer[data.length() + 3], &checksum, sizeof(char));

    //send via udp (length = '$' + data + '#' + 'XX')
    #ifdef RSP_DEBUG 
    std::cout << "send >> " << _output_buffer << std::endl;
    #endif 
    return _server->Send(_output_buffer, data.length() + 4);
}

int RspServer::Receive(char* buffer){

    //check whether the udp server could receive another packet
    int recv_bytes = _server->Receive(buffer);
    if(recv_bytes > 0){

        //check whether the packet is valid
        //TODO: parse checksum
        if(buffer[0] == '$'){

            #ifdef RSP_DEBUG
            buffer[recv_bytes / 2 + 2] = '\0';
            std::cout << "recved << "  << buffer << std::endl;
            #endif

            //acknowledge the receive packet to gdb
            strcpy(_output_buffer, buffer);
            _output_buffer[0] = '+';
            _server->Send(_output_buffer, recv_bytes);
            #ifdef RESP_DEBUG
            std::cout << "ack >> " << buffer << std::endl;
            #endif



            //thread message accordingly
            //message handler depends on the first character
            switch(buffer[0]){

                case 'v':
                    std::cout << "recd v" << std::endl;
		            break;

                //gdb is informing supported extensions
                //case 'v':
                //    this->Respond(std::string(""));
                //    break;

                //commands not implemented by the server
                //must respond with the empty response "$#00".
                default:
                    strcpy(_output_buffer, RSP_EMPTY_RESPONSE);
                    #ifdef RSP_DEBUG
                    std::cout << RSP_EMPTY_RESPONSE << std::endl;
                    #endif
                    return _server->Send(_output_buffer, 5);
                    break;
            }

        }else{
            #ifdef RSP_DEBUG
            std::cout << "ignored << "  << buffer << std::endl;
            #endif
        }
    }

    return -1; //TODO: enum for statuses (could not recv a pkt)


/*
        //continue (unlocks execution until next breakpoin is reached) 
        case 'c': _pcore->Rsv_C(_buffer[1]); break;

        //step: advances PC to the next position
        case 's': _pcore->Rsv_s(_buffer[1]); break;

        //detach GDB from the mote system        
        case 'D': _pcore->Resv_D(_buffer[1]); break;`

        //read general purpose registers (from regbank)
        case 'g': _pcore->Rsv_G(_buffer[1]); break;

        //write general purpose register (to regbank)
        case 'G': _pcore->Rsv_c(_buffer[1]); break;

        //stop software execution (?) 
        //TODO: investigate whether we should adapt it to stop the whole simulation
        case 't':

        //the Z comannd inserts a breakpoint in given address. 
        case 'Z': _pcore->Rsv_z(_buffer[1]); break;

        //the z command removed a breakpoint from given address.
        case 'z': _pcore->Rsv_Z(_buffer[1]); break;
*/

}


int main(){

    char buffer[5000];
    RspServer* srv = new RspServer("localhost", 5000);

    while(1){
        srv->Receive(buffer);
    }


}
