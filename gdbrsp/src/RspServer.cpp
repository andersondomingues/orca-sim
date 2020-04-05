#include <RspServer.h>

RspServer::RspServer(std::string ipaddr, uint32_t udpport, uint16_t port, RspPCore* core){
    
    _udpport = udpport;
    _ipaddr  = ipaddr;

    //create udp server
    _server = new UdpServer(ipaddr, udpport);
    
    //store processor core reference
    _pcore = p;
    
    //create a new udp socket with the informed parameters

    //register OnReceive as the listener

    //start listening
}


RsvServerListener::~ResServerListener(){
    //free srv resources
}

RsvServerListener::OnReceive(){
/*
    //message handler depends on the first character
    switch(_buffer[0]){

        //halt packet
        case '?': _pcore->Rsv_Question(_buffer[1]); break;
       
        //continue (unlocks execution until next breakpoin is reached) 
        case 'c': _pcore->Rsv_C(_buffer[1]); break;

        //step: advances PC to the next position
        case 's': _pcore->Rsv_s(_buffer[1]); break;

        //detach GDB from the mote system        
        case 'D': _pcore->Resv_D(_buffer[1]); break;

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

        default:
            //reply WTF
            break;

    }

*/
}
