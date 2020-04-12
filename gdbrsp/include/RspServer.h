#ifndef _RSP_SERVER_H
#define _RSP_SERVER_H

#include <UdpAsyncServer.h>
#include <iostream>

using namespace std;

#define RSP_BUFFER_SIZE 1000
//#define RSP_EMPTY_RESPONSE "$#00\0" 
#define RSP_EMPTY_RESPONSE ""

class RspServer{

private:
    //upd server instance, see UdpServe.h
    UdpAsyncServer* _server;

    //ip address and udp port for THIS processor
    std::string _ipaddr;
    int _udpport;

    char _output_buffer[RSP_BUFFER_SIZE];
    char _input_buffer[RSP_BUFFER_SIZE];


    //protocol-specific flags
    char _rsp_noack_mode = 0; //starts with noack disabled


public:
    /**
     * Ctor.
     */
    RspServer(std::string ipaddr, uint32_t udpport);   
    
    /**
     * Dtor.
     */
    ~RspServer();
   
    /**
     * Query for a message. 
     */

    int Ack();
    int Nack();

    int Receive(char* buffer);

    uint8_t Checksum(char* buffer, int size);
    int Respond(std::string data);

    /**
     * Message handlers
     */
    int Handle_v(char*);
    int Handle_q(char*);
    int Handle_Q(char*);
    int Handle_Question(char*);
    int Handle_c(char*);
    int Handle_C(char*);
    int Handle_s(char*);
    int Handle_S(char*);
    int Handle_H(char*);
    int Handle_k(char*);
    int Handle_m(char*);
    int Handle_M(char*);
    int Handle_p(char*);
    int Handle_P(char*);
    int Handle_Z(char*);
    int Handle_z(char*);
};

#endif /* _RSP_SERVER_H */
