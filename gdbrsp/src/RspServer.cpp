#include <RspServer.h>

#define RSP_DEBUG 1

template <typename T>
RspServer<T>::RspServer(std::string ipaddr, uint32_t udpport){
    
    _udpport = udpport;
    _ipaddr  = ipaddr;

    //create udp server
    _server = new UdpAsyncServer(udpport);
    std::cout << "[" << udpport << "]";
}

//remove udp server instance
template <typename T>
RspServer<T>::~RspServer(){
    delete _server;
}

//checksum
template <typename T>
uint8_t RspServer<T>::Checksum(char* buffer, int length)
{
    uint8_t checksum = 0;

	while (length--) 
		checksum += *buffer++;
	
	return checksum;
}

template <typename T>
int RspServer<T>::Respond(std::string data){
    
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

template <typename T>
int RspServer<T>::Ack(){

    #ifdef RSP_DEBUG
    std::cout << "\033[0;36m<-- ack\033[0m" << std::endl;
    #endif

    _output_buffer[0] = '+';
    return _server->Send(_output_buffer, 1);
}

template <typename T>
int RspServer<T>::Nack(){
    _output_buffer[0] = '-';
    return _server->Send(_output_buffer, 1);
}

template <typename T>
int RspServer<T>::Receive(ProcessorState<T>* state){

    //check whether the udp server could receive another packet
    int recv_bytes = _server->Receive(_input_buffer);

	if(state->pc == 12345){
		std::cout << "fraafd" << state->pc << std::endl;
	}

    //check whether some packet have been received 
    if(recv_bytes > 0){

        //add termination to received string
        _input_buffer[recv_bytes] = '\0';

        //supports non-ack mode only, ignoring acks
        if(_input_buffer[0] == '+'){
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31m-->\033[0m"  << _input_buffer << std::endl;
            #endif

        //supports nin-ack mode only, ignoring retransmit requests
        }else if(_input_buffer[0] == '-'){
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31m-->\033[0m"  << _input_buffer << std::endl;
            #endif
        //check whether the packet is valid
        //TODO: parse checksum
        }else if(_input_buffer[0] == '$'){

            #ifdef RSP_DEBUG
            std::cout << "\033[0;36m-->\033[0m"  << _input_buffer << std::endl;
            #endif

            //acknowledge if noack mode is inactive
            if(_rsp_noack_mode == 0)
                this->Ack();

            //thread message accordingly
            //message handler depends on the first character
            switch(_input_buffer[1]){

                //query packages to ask the stub for env vars
                //supports only qC, qSupported, qOffset, and qSymbol
                case 'q': return this->Handle_q(_input_buffer);
                case 'Q': return this->Handle_Q(_input_buffer);

                //report why the target halted
                case '?': return this->Handle_Question(_input_buffer);

                //continue and stop commands
                case 'c': return this->Handle_c(_input_buffer);
                case 'C': return this->Handle_C(_input_buffer);
                case 's': return this->Handle_s(_input_buffer);
                case 'S': return this->Handle_S(_input_buffer);

                //TODO: see qC
                case 'H': return this->Handle_H(_input_buffer);

                //kill the target
                case 'k': return this->Handle_k(_input_buffer);

                //memory writing and reading
                case 'm': return this->Handle_m(_input_buffer);
                case 'M': return this->Handle_M(_input_buffer);

                //register writing and reading
                case 'p': return this->Handle_p(_input_buffer);
                case 'P': return this->Handle_P(_input_buffer);

                //vCont, reply empty
                case 'v': return this->Handle_v(_input_buffer);

                //set or clear breakpoints
                case 'z': return this->Handle_z(_input_buffer);
                case 'Z': return this->Handle_Z(_input_buffer);

                //commands not implemented by the server
                //must respond with the empty response "$#00".
                default:

                    #ifdef RSP_DEBUG
                    std::cout << "\033[0;31mwnr: unhandled message: \033[0m" << _input_buffer << std::endl;
                    #endif

                    //return this->Respond(RSP_EMPTY_RESPONSE);
                    _output_buffer[0] = '\0';
                    return _server->Send(_output_buffer, 0);
            }

        }else{
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31mwrn: dropped unknown packet:\033[0m"  << _input_buffer << std::endl;
            #endif
        }
    }

    return -1; //TODO: enum for statuses (could not recv a pkt)
}

template <typename T>
int RspServer<T>::Handle_v(char* buffer){

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
template <typename T>
int RspServer<T>::Handle_Q(char* buffer){

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
template <typename T>
int RspServer<T>::Handle_q(char* buffer){
    
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

template <typename T>
int RspServer<T>::Handle_Question(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_C(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_c(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_s(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_S(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_H(char*){
    return this->Ack();
}

template <typename T>
int RspServer<T>::Handle_k(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_m(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_M(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_p(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_P(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_Z(char*){
    return 0;
}

template <typename T>
int RspServer<T>::Handle_z(char*){
    return 0;
}

