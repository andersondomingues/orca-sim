#include <RspServer.h>
#include <sstream>
#include <iomanip>

#define RSP_DEBUG 1

template <typename T>
RspServer<T>::RspServer(ProcessorState<T>* state, UMemory* mem, std::string ipaddr, uint32_t udpport){
    
    _udpport = udpport;
    _ipaddr  = ipaddr;

	//store external state reference and memory
	_state = state;
	_memory = mem;

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
    uint8_t checksum = this->Checksum(&_output_buffer[1], data.length());
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
int RspServer<T>::Receive(){

    //check whether the udp server could receive another packet
    int recv_bytes = _server->Receive(_input_buffer);

    //check whether some packet have been received 
    if(recv_bytes > 0){

        //add termination to received string (to guarantee a printable output)
        _input_buffer[recv_bytes] = '\0';

	//received packet is an ACK, ignoring (connection is assumed to be reliable)
        if(_input_buffer[0] == '+'){
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31m-->\033[0m"  << _input_buffer << std::endl;
            #endif

	//retransmission will not be required, ignoring any request
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

            //acknowledge all valid packets
            this->Ack();

            //message handler depends on the first character
            switch(_input_buffer[1]){

                //query packages to ask the stub for env vars
                //qC, qSupported, qOffset, and qSymbol must be supported
                case 'q': return this->Handle_q(_input_buffer);

				//report general registers
				case 'g': return this->Handle_g(_input_buffer);
				
                //case 'Q': return this->Handle_Q(_input_buffer);

                //report why the target halted
                case '?': return this->Handle_Question(_input_buffer);

                //continue and stop commands
                //case 'c': return this->Handle_c(_input_buffer);
                //case 'C': return this->Handle_C(_input_buffer);
                //case 's': return this->Handle_s(_input_buffer);
                //case 'S': return this->Handle_S(_input_buffer);

                //TODO: see qC
                case 'H': return this->Handle_H(_input_buffer);

                //kill the target
                //case 'k': return this->Handle_k(_input_buffer);

                //memory writing and reading
               	case 'm': return this->Handle_m(_input_buffer);
                //case 'M': return this->Handle_M(_input_buffer);

                //register writing and reading
                case 'p': return this->Handle_p(_input_buffer);
                case 'P': return this->Handle_P(_input_buffer);

                //vCont, reply empty
                case 'v': return this->Handle_v(_input_buffer);

                //set or clear breakpoints
                //case 'z': return this->Handle_z(_input_buffer);
                //case 'Z': return this->Handle_Z(_input_buffer);

                //commands not implemented by the server
                //must respond with the empty response "$#00".
                default:

                    #ifdef RSP_DEBUG
                    std::cout << "\033[0;31mwnr: packet unhandled: \033[0m" << _input_buffer << std::endl;
                    #endif

                    //return this->Respond(RSP_EMPTY_RESPONSE);
                    _output_buffer[0] = '\0';
                    return _server->Send(_output_buffer, 0);
            }

        }else{
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31mwrn: dropped packet with unknown prefix:\033[0m"  << _input_buffer << std::endl;
            #endif
        }
    }

    return -1; //TODO: enum for statuses (could not recv a pkt)
}



template <typename T>
int RspServer<T>::Handle_g(char* buffer){

	//there is only one 'g' message, just report registers
	if(memcmp(buffer, "$g", 2) == 0){

		//prints as [0x]00000000, number of regs * size of reg * size of char
		char reg_data[NUMBER_OF_REGISTERS * sizeof(T) * 2];

		//add data for all registers
		//TODO: format print according to register size
		for(int i = 0; i < NUMBER_OF_REGISTERS; i++)
			sprintf(&reg_data[i * sizeof(T) * 2], "%08X", (unsigned int)_state->regs[i]);

		//send regs info
		//return this->Respond(reg_data);
		return this->Respond(reg_data);

	}else{
		std::cout << "unhandled packet 'g'" << std::endl;
	}
	
	return -1;
}

template <typename T>
int RspServer<T>::Handle_v(char* buffer){

    //vCont, must reply empty (not supported)
    if(memcmp(buffer, "$vCont", 5) == 0){
        return this->Respond(RSP_EMPTY_RESPONSE);
    
	//vKill, gdb ask to stop debugging
	//TODO: set state to "not running"
    }else if(memcmp(buffer, "$vKill", 5) == 0){
        return this->Respond("OK");
    
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
        this->Respond("OK");
    }else{
        std::cout << "unhandled packet 'Q'"  << std::endl;
    }

	return 0;
}

//query packets (lower case Q is for GET)
template <typename T>
int RspServer<T>::Handle_q(char* buffer){
    
    //qC is not supported
    if(memcmp(buffer, "$qC", 3) == 0){
        return this->Respond("QC1");
	
	//is always attached
	}else if(memcmp(buffer, "$qAttached", 7) == 0){
		return this->Respond("0");
		//return this->Respond("");

	//target doesn't move the offsets, reply all zero
    }else if(memcmp(buffer, "$qOffsets", 6) == 0){
		return this->Respond("Text=0;Data=0;Bss=0");
	
	//notify target that client can provide symbols
    }else if(memcmp(buffer, "$qSymbol", 6) == 0){
		return this->Respond("OK");

    //report supported packet size and request to diable ack mode
    }else if(memcmp(buffer, "$qSupported", 11) == 0){
		return this->Respond("PacketSize=512"); //QStartNoAckMode+

	//currently running threads (one only)
	}else if(memcmp(buffer, "$qfThreadInfo", 11) == 0){
		//return this->Respond("m 0"); //alternatively "m1"
		return this->Respond("m1"); //alternatively "m1"

	//report the end of list (see qfThreadInfo)
	}else if(memcmp(buffer, "$qsThreadInfo", 11) == 0){
		return this->Respond("l");

    //report thread status
    }else if(memcmp(buffer, "$qTStatus", 8) == 0){       
		return this->Respond("T0");

	//qTfv and qTsv
    }else{
		std::cout << "unhandled qT packet, sent empty response" << std::endl;
		return this->Respond("");
	}

    return -1;
}

//why did the program stop?
template <typename T>
int RspServer<T>::Handle_Question(char*){
    return this->Respond("S00"); // <- exited ok
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

//select thread for subsequent actions, thread-0
//is always selected
template <typename T>
int RspServer<T>::Handle_H(char*){

    return this->Respond("OK");
}

template <typename T>
int RspServer<T>::Handle_k(char*){
    return 0;
}

//'m': read value from the main memory
//format => "$mX,Y#CC", where X is the
//address, Y is the size
template <typename T>
int RspServer<T>::Handle_m(char* buffer){
	
	//only one packet type 'm'
	if(memcmp(buffer, "$m", 2) == 0){

		uint32_t addr = 0, size = 0;
		int end = 0, comma = 0;

		//can crash here if no "#" in the message
		//TODO: make sure that at least one "#" is present
		end = strfind(buffer, '#', 100);

		//raddr starts after the 'm' character and
		//goes until the ','
		addr = strhti(&buffer[2], end - 2);

		//find the ',' char
		comma = strfind(buffer, ',', end); 

		//find size. NOTE: size means "X 16-bit words"
		size = strhti(&buffer[comma+1], end) * 2;

		//@TODO: why does GBD ask for addr=0?
		//@TODO: remove restriction on 8bit memory model (another T param?)
		if(addr < _memory->GetBase() || addr > _memory->GetLastAddr()){
			std::cout << "gdb client requested data from outside memory space, ignoring" << std::endl;			
			return this->Respond(""); //return 8 bits char 
		}

		//read data from memory
		MemoryType data[size * sizeof(MemoryType)];
		_memory->Read(addr, data, size);

		//assuming bytes is always even. add size of int to avoid out of bounds
		MemoryType str_data[size * sizeof(MemoryType) + sizeof(int)];

		//convert from byte to hexstring
		hexstr((char*)str_data, (char*)data, size * sizeof(MemoryType));

		//add string termination
		*((char*)&str_data[size * sizeof(MemoryType)]) = '\0'; 

		return this->Respond(std::string((char*)str_data));

	}else{

		std::cout << "unhandled 'm' packet, sent empty response" << std::endl;
		return this->Respond("");
	}
	
    return -1;
}

template <typename T>
int RspServer<T>::Handle_M(char*){
    return 0;
}

//read registers
template <typename T>
int RspServer<T>::Handle_p(char* buffer){

	//only one 'p' type packet
	if(memcmp(buffer, "$p", 2) == 0){
		
		//parse register id 
		char digit[4] = "\0\0\0";

		//get the two digits with hexa-code reg id
		if(buffer[2] >= 48 && buffer[2] <= 57) digit[0] = buffer[2];
		if(buffer[3] >= 48 && buffer[3] <= 57) digit[1] = buffer[3];

		//convert ascii hexa to integer, so we can access the array
		unsigned int reg = (int)strtol(digit, NULL, 16);
		
		//allocate space for the ascii hexa containing the value
		char reg_data[sizeof(T) * 2 + 1]; //we add 1 because sprintf adds \0 to the end

		//print the value as hexa string into the allocated space
		sprintf(reg_data, "%0X", (unsigned int)_state->regs[reg]);

		std::string red_data_without_last_character = std::string(reg_data);
		printf("%d", (int)red_data_without_last_character.length());

		this->Respond(red_data_without_last_character);

	}else{
		std::cout << "unhandled 'p' packet, sent empty response" << std::endl;
		return this->Respond("");
	}
    return -1;
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

//string hex to int
int strhti(char* buffer, int length){
	
	char tmp[length];

	for(int i = 0; i < length; i++)
		tmp[i] = '\0';

	for(int i =0; i < length; i++){
		//is [0-9] digit
		if(buffer[i] >= 48 && buffer[i] <= 57){
			tmp[i] = buffer[i];
		//is [a-f] digit
		}else if(buffer[i] >= 97 && buffer[i] <= 122){
			tmp[i] = buffer[i];
		}else{
			break;
		}
	}

	return (int)strtol(tmp, NULL, 16);
}

//find first occurrence of a character, returns index
int strfind(char* buffer, char find, int limit){

	for(int i = 0; i < limit; i++)
		if(buffer[i] == find) return i;

	return -1;
}

//byte array to hexa
void hexstr(char* output, char* input, uint32_t bytes){
	
	printf("hexstr:");

	uint32_t k, l;
	
	//mask is necessary to correct a bug(?) 
	//when printing negative values.
	uint32_t mask = 0x000000FF; 
	
	for(k = 0; k < bytes; k += 16){
		for(l = 0; l < 16; l++){
			sprintf(&output[k + l], "%02x", input[k + l] & mask );
			printf("%02x", input[k + l] & mask );
		}
	}

	//@TODO:does gdb require endianess treatment?

	printf("\n");
}

