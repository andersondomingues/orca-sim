#include <RspServer.h>

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
    

    //copy data into output buffer
    memcpy(_output_buffer, data.c_str(), data.length());

    //calculate checksum
    uint8_t checksum = this->Checksum(_output_buffer, data.length());

    //append checksum to the data
    memcpy(&_output_buffer[data.length()], &checksum, sizeof(char));

    //add string termination charater
    _output_buffer[data.length() + 2] = '\0';

    //send via udp
    return _server->Send(_output_buffer, data.length() + 3);
}

int RspServer::Receive(char* buffer){

    //check whether the udp server could receive another packet
    if(_server->Receive(buffer) > 0){


        std::cout << buffer << std::endl;

        //thread message accordingly
        //message handler depends on the first character
        switch(buffer[0]){

            //gdb is informing supported extensions
            case '$':
                this->Respond(std::string("$swbreak"));

            //halt packet
            case '?': 
                //_pcore->Rsv_Question(_buffer[1]); break;
                break;
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
            default:
                //reply WTF
                break;
        }
    }

    return 1;

}


int main(){

    char buffer[5000];
    RspServer* srv = new RspServer("localhost", 5000);

    while(1){
        srv->Receive(buffer);
    }


}
