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

    //copy data into output buffer ( -1 eliminates terminating character)
    memcpy(&_output_buffer[1], data.c_str(), data.length());

    //add separator to the ourput
    _output_buffer[data.length() + 1] = '#';

    //calculate checksum and append it to the pkt
    uint8_t checksum = this->Checksum(_output_buffer, data.length());
    sprintf((char*)&_output_buffer[data.length() + 2], "%02x", checksum);

    //add terminating charaters to the string
    _output_buffer[data.length() + 4] = '\0';

    #ifdef RSP_DEBUG 
    std::cout << "\033[0;36m<--\033[0m" << _output_buffer << std::endl;
    #endif 

     //send via udp (length = '$' + data + '#' + 'XX')
    return _server->Send(_output_buffer, data.length() + 4);
}

int RspServer::Ack(){

    #ifdef RSP_DEBUG
    std::cout << "\033[0;36m<-- ack\033[0m" << std::endl;
    #endif

    _output_buffer[0] = '+';
    return _server->Send(_output_buffer, 1);
}

int RspServer::Nack(){
    _output_buffer[0] = '-';
    return _server->Send(_output_buffer, 1);
}

int RspServer::Receive(char* buffer){

    //check whether the udp server could receive another packet
    int recv_bytes = _server->Receive(buffer);

    //check whether some packet have been received 
    if(recv_bytes > 0){

        //add termination to received string
        buffer[recv_bytes] = '\0';

        //supports non-ack mode only, ignoring acks
        if(buffer[0] == '+'){
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31m-->\033[0m"  << buffer << std::endl;
            #endif

        //supports nin-ack mode only, ignoring retransmit requests
        }else if(buffer[0] == '-'){
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31m-->\033[0m"  << buffer << std::endl;
            #endif
        //check whether the packet is valid
        //TODO: parse checksum
        }else if(buffer[0] == '$'){

            #ifdef RSP_DEBUG
            std::cout << "\033[0;36m-->\033[0m"  << buffer << std::endl;
            #endif

            //acknowledge if noack mode is inactive
            if(_rsp_noack_mode == 0)
                this->Ack();

            //thread message accordingly
            //message handler depends on the first character
            switch(buffer[1]){

                //query packages to ask the stub for env vars
                //supports only qC, qSupported, qOffset, and qSymbol
                case 'q': return this->Handle_q(buffer);
                case 'Q': return this->Handle_Q(buffer);

                //report why the target halted
                case '?': return this->Handle_Question(buffer);

                //continue and stop commands
                case 'c': return this->Handle_c(buffer);
                case 'C': return this->Handle_C(buffer);
                case 's': return this->Handle_s(buffer);
                case 'S': return this->Handle_S(buffer);

                //TODO: see qC
                case 'H': return this->Handle_H(buffer);

                //kill the target
                case 'k': return this->Handle_k(buffer);

                //memory writing and reading
                case 'm': return this->Handle_m(buffer);
                case 'M': return this->Handle_M(buffer);

                //register writing and reading
                case 'p': return this->Handle_p(buffer);
                case 'P': return this->Handle_P(buffer);

                //vCont, reply empty
                case 'v': return this->Handle_v(buffer);

                //set or clear breakpoints
                case 'z': return this->Handle_z(buffer);
                case 'Z': return this->Handle_Z(buffer);

                //commands not implemented by the server
                //must respond with the empty response "$#00".
                default:

                    #ifdef RSP_DEBUG
                    std::cout << "\033[0;31mwnr: unhandled message: \033[0m" << buffer << std::endl;
                    #endif

                    //return this->Respond(RSP_EMPTY_RESPONSE);
                    _output_buffer[0] = '\0';
                    return _server->Send(_output_buffer, 0);
            }

        }else{
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31mwrn: dropped unknown packet:\033[0m"  << buffer << std::endl;
            #endif
        }
    }

    return -1; //TODO: enum for statuses (could not recv a pkt)
}

int RspServer::Handle_v(char* buffer){

    //vCont, must reply empty
    if(strcmp(&buffer[1], "vCont") == 0){
        this->Respond(RSP_EMPTY_RESPONSE);
    
    //vMustReplyEmpty, must reply empty
    } else if(memcmp(buffer, "$vMustReplyEmpty", 16) == 0){
        return this->Respond(RSP_EMPTY_RESPONSE);

    //unhandled messages
    } else {
        std::cout << "unhandled packet 'v'" << std::endl;
    }

	return 1;
}

//query packets (upper case Q is for SET)
int RspServer::Handle_Q(char* buffer){

    //QStartNoAckMode, disables aknowledgement messages (+)
    if(memcmp(buffer, "$QStartNoAckMode", 15) == 0){
        _rsp_noack_mode = 1;
        this->Respond("OK");
    }else{
        std::cout << "unhandled packet 'Q'"  << std::endl;
    }

	return 0;
}

//query packets (lower case Q is for GET)
int RspServer::Handle_q(char* buffer){
    
    //supports only qC, qSupported, qOffset, and qSymbol
    if(memcmp(buffer, "$qC", 3) == 0){
        return this->Respond(RSP_EMPTY_RESPONSE);

    }else if(strcmp(buffer, "$qOffset") == 0){

    }else if(strcmp(buffer, "$qSymbol") == 0){

    //report supported packet size and request to diable ack mode
    }else if(memcmp(buffer, "$qSupported", 11) == 0){
        return this->Respond("PacketSize=256;QStartNoAckMode+");

    //report thread status
    }else if(strcmp(buffer, "$qTStatus") == 0){
        return this->Respond("tnorun:0");
    }

    return -1;
}

int RspServer::Handle_Question(char*){
    return 0;
}

int RspServer::Handle_C(char*){
    return 0;
}

int RspServer::Handle_c(char*){
    return 0;
}

int RspServer::Handle_s(char*){
    return 0;
}

int RspServer::Handle_S(char*){
    return 0;
}

int RspServer::Handle_H(char*){
    return this->Ack();
}

int RspServer::Handle_k(char*){
    return 0;
}

int RspServer::Handle_m(char*){
    return 0;
}

int RspServer::Handle_M(char*){
    return 0;
}

int RspServer::Handle_p(char*){
    return 0;
}

int RspServer::Handle_P(char*){
    return 0;
}

int RspServer::Handle_Z(char*){
    return 0;
}

int RspServer::Handle_z(char*){
    return 0;
}

/*int main(){

    char buffer[5000];
    RspServer* srv = new RspServer("127.0.0.1", 5000);

    while(1){
        srv->Receive(buffer);
    }


}*/
