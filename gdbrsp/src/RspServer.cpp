/******************************************************************************
 * This file is part of project ORCA. More information on the project
 * can be found at the following repositories at GitHub's website.
 *
 * http://https://github.com/andersondomingues/orca-sim
 * http://https://github.com/andersondomingues/orca-software
 * http://https://github.com/andersondomingues/orca-mpsoc
 * http://https://github.com/andersondomingues/orca-tools
 *
 * Copyright (C) 2018-2020 Anderson Domingues, <ti.andersondomingues@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. 
******************************************************************************/
#include <sstream>

#include "RspServer.hpp"
#include "DataConvertionHelper.hpp"

#define RSP_DEBUG 1

template <typename T>
RspServer<T>::RspServer(ProcessorState<T>* state, UMemory* mem,
    std::string ipaddr, uint32_t udpport) {

    _udpport = udpport;
    _ipaddr  = ipaddr;

    // store external state reference and memory
    _state = state;
    _memory = mem;

    // create udp server
    _server = new UdpAsyncServer(udpport);
    std::cout << "[" << udpport << "]";

    _bp_list = new std::list<T>();
}

// remove udp server instance
template <typename T>
RspServer<T>::~RspServer() {
    delete _server;
}

// checksum
template <typename T>
uint8_t RspServer<T>::Checksum(char* buffer, int length) {
    uint8_t checksum = 0;

    while (length--)
        checksum += *buffer++;

    return checksum;
}

template <typename T>
int RspServer<T>::Respond(std::string data) {
    // packet response should be "$data#checksum"
    // copy the $ charater to the output
    _output_buffer[0] = '$';

    // copy data into output buffer ( -1 eliminates terminating character)
    memcpy(&_output_buffer[1], data.c_str(), data.length());

    // add separator to the ourput
    _output_buffer[data.length() + 1] = '#';

    // calculate checksum and append it to the pkt
    uint8_t checksum = this->Checksum(&_output_buffer[1], data.length());
    snprintf(reinterpret_cast<char*>(&_output_buffer[data.length() + 2]),
     sizeof(uint32_t), "%02x", checksum);

    // add terminating charaters to the string
    _output_buffer[data.length() + 4] = '\0';

    #ifdef RSP_DEBUG
    std::cout << "\033[0;36m<--\033[0m" << _output_buffer << std::endl;
    #endif

    // send via udp (length = '$' + data + '#' + 'XX')
    return _server->Send(_output_buffer, data.length() + 4);
}

template <typename T>
int RspServer<T>::Ack() {
    #ifdef RSP_DEBUG
    std::cout << "\033[0;36m<-- ack\033[0m" << std::endl;
    #endif

    _output_buffer[0] = '+';
    return _server->Send(_output_buffer, 1);
}

template <typename T>
int RspServer<T>::Nack() {
    _output_buffer[0] = '-';
    return _server->Send(_output_buffer, 1);
}

template <typename T>
int RspServer<T>::UpdateCpuState() {
    // check whether we reach a breakpoint
    if (_state->bp == 0) {
        for (typename std::list<T>::iterator i = _bp_list->begin();
            i != _bp_list->end(); ++i) {
            if (_state->pc == *i) {
                // breakpoint hit
                // this->Respond("T05swbreak:");
                this->Respond("S05");
                _state->bp = 1;
                break;
            }
        }
    }

    // if CPU reached a breakpoint
    if (_state->bp == 1) {
        // _state->bp = 0;  // clear breakpoint
        _state->pause = 1;  // pause cpu
        _state->steps = 0;  // reset steps counter (bp has priority over s)
        // this->Respond("T05:");

    // if CPU no pause with no bp
    } else if (_state->pause == 0) {
        if (_state->steps > 0) {
            _state->steps -= 1;

            // limit of steps reach, pausing
            if (_state->steps == 0) {
                _state->pause = 1;
                this->Respond("S05");  // 05 = TRAP
            }
        }
    }

    return 1;
}

template <typename T>
int RspServer<T>::Receive() {
    // check whether the udp server could receive another packet
    int recv_bytes = _server->Receive(_input_buffer);

    // check whether some packet have been received
    if (recv_bytes > 0) {
        // add termination to received string, garantees printable output
        _input_buffer[recv_bytes] = '\0';

        // received packet is an ACK, ignoring (if connection is reliable)
        if (_input_buffer[0] == '+') {
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31m-->\033[0m"  << _input_buffer << std::endl;
            #endif

        // retransmission will not be required, ignoring any request
        } else if (_input_buffer[0] == '-') {
            #ifdef RSP_DEBUG
            std::cout << "\033[0;31m-->\033[0m"  << _input_buffer << std::endl;
            #endif

        // check whether the packet is valid
        // TODO(ad): parse checksum
        } else if (_input_buffer[0] == '$') {
            #ifdef RSP_DEBUG
            std::cout << "\033[0;36m-->\033[0m"  << _input_buffer << std::endl;
            #endif

            // acknowledge all valid packets
            this->Ack();

            // message handler depends on the first character
            switch (_input_buffer[1]) {
                case 'q': return this->Handle_q(_input_buffer);  // query pckt
                case 'g': return this->Handle_g(_input_buffer);  // get regs
                case '?': return this->Handle_Question(_input_buffer);  // hlt
                case 'c': return this->Handle_c(_input_buffer);  // continue
                case 'C': return this->Handle_C(_input_buffer);  // continue
                case 's': return this->Handle_s(_input_buffer);  // continue
                case 'H': return this->Handle_H(_input_buffer);  // Hc
                case 'm': return this->Handle_m(_input_buffer);  // mem read
                // case 'M': return this->Handle_M(_input_buffer); // mem write
                case 'p': return this->Handle_p(_input_buffer);  // reg read
                case 'P': return this->Handle_P(_input_buffer);  // reg write
                case 'v': return this->Handle_v(_input_buffer);  // vCont
                case 'X': return this->Handle_X(_input_buffer);  // bin mem wr
                case 'z': return this->Handle_z(_input_buffer);  // set bp
                case 'Z': return this->Handle_Z(_input_buffer);  // clear bp
                // case 'Q': return this->Handle_Q(_input_buffer);

                // commands not implemented by the server
                // must responded with the empty response "$#00".
                default:

                    #ifdef RSP_DEBUG
                    std::cout << "\033[0;31mwnr: unknown packet type: \033[0m"
                        << _input_buffer << std::endl;
                    #endif

                    // return this->Respond(RSP_EMPTY_RESPONSE);
                    return this->Respond("");
            }
        } else {
            #ifdef RSP_DEBUG
            std::cout
                << "\033[0;31mwrn: dropped packet with unknown prefix:\033[0m"
                << _input_buffer << std::endl;
            #endif
        }
    }

    return -1;  // TODO(ad): enum for statuses (could not recv a pkt)
}

template <typename T>
int RspServer<T>::Handle_X(char* buffer) {
    if (memcmp(buffer, "$X", 2) == 0) {
        // locate addr to start writing to
        int addr = strhti(&buffer[2], 10);

        // find number of bytes to write
        int i = strfind(buffer, ',', 14);
        int size = strhti(&buffer[i+1], 10);

        // respond ok if test message
        if (size == 0) {
            return this->Respond("OK");

        // otherwise, write to memory
        } else {
            // find the beggining of the stream
            i = strfind(buffer, ':', 20);
            uint8_t* raw = reinterpret_cast<uint8_t*>(&buffer[i+1]);

            // note: binary data comes from GDB
            // client with bytes in the correct
            // order, so no endianess treatment is
            // necessary.

            // remove escape chars (0x7d = '}')
            for (int j = 0, i = 0; i < size; i++) {
                if (raw[j] == 0x7d) {
                    raw[i] = (uint8_t)(raw[++j] ^ 0x20);
                    j++;
                } else {
                    raw[i] = raw[j++];
                }
            }

            // write the whole chunk to the memory of the device
            _memory->Write(addr, reinterpret_cast<MemoryType*>(raw), size);
            return this->Respond("OK");
        }
    } else {
        std::cout << "unhandled packet 'X'" << std::endl;
    }

    return -1;
}

template <typename T>
int RspServer<T>::Handle_g(char* buffer) {
    // there is only one 'g' message, just report registers
    if (memcmp(buffer, "$g", 2) == 0) {
        // prints as [0x]00000000,
        // (number of regs + pc) * size of reg * 2 char per byte + '\0'
        char reg_data[(NUMBER_OF_REGISTERS + 1) * sizeof(T) * 2 + 1];

        // convert whole array to hexstring
        hexstr(reinterpret_cast<char*>(reg_data),
            reinterpret_cast<char*>(_state->regs), NUMBER_OF_REGISTERS + 1);

        // do not add trailing character as hexstr adds it
        return this->Respond(std::string(reg_data));

    } else {
        std::cout << "unhandled packet 'g'" << std::endl;
    }

    return -1;
}

template <typename T>
int RspServer<T>::Handle_v(char* buffer) {
    // vCont, must reply empty (not supported)
    if (memcmp(buffer, "$vCont", 5) == 0) {
        return this->Respond(RSP_EMPTY_RESPONSE);

    // vKill, gdb ask to stop debugging
    // TODO(ad): set state to "not running"
    } else if (memcmp(buffer, "$vKill", 5) == 0) {
        this->Respond("OK");
        exit(0);  // TODO(ad): verify whether this is necessary

    // vMustReplyEmpty, must reply empty
    } else if (memcmp(buffer, "$vMustReplyEmpty", 16) == 0) {
        return this->Respond(RSP_EMPTY_RESPONSE);

    // unhandled messages
    } else {
        std::cout << "unhandled packet 'v'" << std::endl;
    }

    return 1;
}

// query packets (upper case Q is for SET)
template <typename T>
int RspServer<T>::Handle_Q(char* buffer) {
    // QStartNoAckMode, disables aknowledgement messages (+)
    if (memcmp(buffer, "$QStartNoAckMode", 15) == 0) {
        this->Respond("OK");
    } else {
        std::cout << "unhandled packet 'Q'"  << std::endl;
    }

    return 0;
}

// query packets (lower case Q is for GET)
template <typename T>
int RspServer<T>::Handle_q(char* buffer) {
    // qC is not supported
    if (memcmp(buffer, "$qC", 3) == 0) {
        return this->Respond("QC1");

    // is always attached
    } else if (memcmp(buffer, "$qAttached", 7) == 0) {
        return this->Respond("0");
        // return this->Respond("");

    // target doesn't move the offsets, reply all zero
    } else if (memcmp(buffer, "$qOffsets", 6) == 0) {
        return this->Respond("Text=0;Data=0;Bss=0");

    // notify target that client can provide symbols
    } else if (memcmp(buffer, "$qSymbol", 6) == 0) {
        return this->Respond("OK");

    // report supported packet size and request to diable ack mode
    } else if (memcmp(buffer, "$qSupported", 11) == 0) {
        return this->Respond("PacketSize=FFF");  // QStartNoAckMode+

    // currently running threads (one only)
    } else if (memcmp(buffer, "$qfThreadInfo", 11) == 0) {
        // return this->Respond("m 0"); //alternatively "m1"
        return this->Respond("m1");  // alternatively "m1"

    // report the end of list (see qfThreadInfo)
    } else if (memcmp(buffer, "$qsThreadInfo", 11) == 0) {
        return this->Respond("l");

    // report thread status
    } else if (memcmp(buffer, "$qTStatus", 8) == 0) {
        return this->Respond("T0");

    // qTfv and qTsv
    } else {
        std::cout << "unhandled qT packet, sent empty response" << std::endl;
        return this->Respond("");
    }

    return -1;
}

// why did the program stop?
template <typename T>
int RspServer<T>::Handle_Question(char*) {
    return this->Respond("S00");  // <- exited ok
}

// continue at the current address
template <typename T>
int RspServer<T>::Handle_C(char* buffer) {
    if (memcmp(buffer, "$C", 2) == 0) {
        // disable pause mode and continue
        // in the same address as it is
        _state->pause = 0x0;
        return this->Respond("OK");
    } else {
        std::cout << "unhandled 'c' packet, sent empty response" << std::endl;
        return this->Respond("");
    }

    return -1;
}

// continue at the current address
template <typename T>
int RspServer<T>::Handle_c(char* buffer) {
    // @TODO: implement "continue at addr"
    if (memcmp(buffer, "$c", 2) == 0) {
        // disable pause mode and continue
        // in the same address as it is
        _state->pause = 0x0;
        return this->Respond("OK");
    } else {
        std::cout << "unhandled 'c' packet, sent empty response" << std::endl;
        return this->Respond("");
    }

    return -1;
}

// step X instructions
template <typename T>
int RspServer<T>::Handle_s(char* buffer) {
    if (memcmp(buffer, "$s", 2) == 0) {
        // single step
        if (buffer[2] == '#') {
            _state->steps = 1;    // set to stop cpu in next cycle
            _state->pause = 0x0;  // remove pause until then
            this->Respond("OK");
        } else {
            std::cout << "multiple step not implemented, sent empty response"
                << std::endl;
            this->Respond("");
        }
    } else {
        std::cout << "unhandled 's' packet, sent empty response" << std::endl;
        this->Respond("");
    }
    return 0;
}

// select thread for subsequent actions, thread-0
// is always selected
template <typename T>
int RspServer<T>::Handle_H(char*) {
    return this->Respond("OK");
}

template <typename T>
int RspServer<T>::Handle_k(char*) {
    return 0;
}

// 'm': read value from the main memory
// format => "$mX,Y#CC", where X is the
// address, Y is the size
template <typename T>
int RspServer<T>::Handle_m(char* buffer) {
    // only one packet type 'm'
    if (memcmp(buffer, "$m", 2) == 0) {
        uint32_t addr = 0, size = 0;
        int end = 0, comma = 0;

        // can crash here if no "#" in the message
        // TODO(ad): make sure that at least one "#" is present
        end = strfind(buffer, '#', 100);

        // raddr starts after the 'm' character and
        // goes until the ','
        addr = strhti(&buffer[2], end - 2);

        // find the ',' char
        comma = strfind(buffer, ',', end);

        // find size. NOTE: size means "X 16-bit words"
        size = strhti(&buffer[comma+1], end);  // * 2;

        // @TODO: why does GBD ask for addr=0?
        // @TODO: remove restriction on 8bit memory model (another T param?)
        if (addr < _memory->GetBase() || addr > _memory->GetLastAddr()) {
            std::cout << "gdbcli requested data from outside memory space:"
                << std::endl;
            return this->Respond("");  // return 8 bits char
        }

        // read data from memory, 0x00 for each byte
        MemoryType data[size * sizeof(MemoryType) * 2];
        _memory->Read(addr, data, size);

        // store hexstring, "00000000" per integer, "00" per byte + '\0'
        char str_data[size * 2 + 8];  // add 8 for integer-alignment

        // convert from byte to hexstring
        hexstr(reinterpret_cast<char*>(str_data),
         reinterpret_cast<char*>(data), size / 2);

        return this->Respond(std::string(reinterpret_cast<char*>(str_data)));
    } else {
        std::cout << "unhandled 'm' packet, sent empty response" << std::endl;
        return this->Respond("");
    }
    return -1;
}

template <typename T>
int RspServer<T>::Handle_M(char*) {
    return 0;
}

// read registers
template <typename T>
int RspServer<T>::Handle_p(char* buffer) {
    // only one 'p' type packet
    if (memcmp(buffer, "$p", 2) == 0) {
        // parse register id
        char digit[4] = "\0\0\0";

        // get the two digits with hexa-code reg id
        if (buffer[2] >= 48 && buffer[2] <= 57) digit[0] = buffer[2];
        if (buffer[3] >= 48 && buffer[3] <= 57) digit[1] = buffer[3];

        // convert ascii hexa to integer, so we can access the array
        unsigned int reg = static_cast<int>(strtol(digit, NULL, 16));

        // allocate space for the ascii hexa containing the value
        // we add 1 because sprintf adds \0 to the end
        char reg_data[sizeof(T) * 2 + 1];

        // print the value as hexa string into the allocated space
        snprintf(reg_data, sizeof(reg_data), "%0X",
            (unsigned int)_state->regs[reg]);

        this->Respond(std::string(reg_data));

    } else {
        std::cout << "unhandled 'p' packet, sent empty response" << std::endl;
        return this->Respond("");
    }
    return -1;
}

// write register
template <typename T>
int RspServer<T>::Handle_P(char* buffer) {
    if (memcmp(buffer, "$P", 2) == 0) {
        // locate register address
        int addr = strhti(&buffer[2], 3);

        // locate the '=' symbol
        int eqsymb = strfind(buffer, '=', 10);

        // get the register value
        int value = strhti(&buffer[eqsymb+1], 10);
        value = endswap(value);

        // set register
        _state->regs[addr] = value;
        return this->Respond("OK");

    } else {
        std::cout << "unhandled 'P' packet, sent empty response" << std::endl;
        return this->Respond("");
    }

    return -1;
}


// adds a break point, only software supported
template <typename T>
int RspServer<T>::Handle_Z(char* buffer) {
    if (memcmp(buffer, "$Z", 2) == 0) {
        // locate the comma char
        int comma = strfind(buffer, ',', 10);

        // parse address
        T addr = strhti(&buffer[comma + 1], 10);

        // check whether the address is in the list already
        bool has_addr_already = false;

        for (typename std::list<T>::iterator i = _bp_list->begin();
            i != _bp_list->end(); ++i) {
            if (addr == *i) {
                std::cout
                    << "warn: there's a breakpoint in this address already"
                    << std::endl;
                has_addr_already = true;
                break;
            }
        }

        // add address to the list
        if (!has_addr_already) _bp_list->push_back(addr);

        // !!ignoring breakpoint size, not supported by arch
        // !!treating all bp as software bp, hw bp not supported
        return this->Respond("OK");

    } else {
        std::cout << "unhandled 'Z' packet, sent empty response" << std::endl;
        return this->Respond("");
    }

    return -1;
}

// removes a breakpoint, only software supported
template <typename T>
int RspServer<T>::Handle_z(char* buffer) {
    if (memcmp(buffer, "$z", 2) == 0) {
        // locate the comma char
        int comma = strfind(buffer, ',', 10);

        // parse address
        T addr = strhti(&buffer[comma + 1], 10);

        // remove the address from the list (if there)
        _bp_list->remove(addr);

        // !!ignoring breakpoint size, not supported by arch
        // !!treating all bp as software bp, hw bp not supported
        return this->Respond("OK");

    } else {
        std::cout << "unhandled 'z' packet, sent empty response" << std::endl;
        return this->Respond("");
    }

    return 0;
}
