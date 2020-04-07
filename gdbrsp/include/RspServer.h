#ifndef _RSP_SERVER_H
#define _RSP_SERVER_H

#include <UdpAsyncServer.h>
#include <iostream>

using namespace std;

#define RSP_BUFFER_SIZE 1000


class RspServer{

private:
    //upd server instance, see UdpServe.h
    UdpAsyncServer* _server;

    //ip address and udp port for THIS processor
    std::string _ipaddr;
    int _udpport;

    char _output_buffer[RSP_BUFFER_SIZE];
    char _input_buffer[RSP_BUFFER_SIZE];

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
     * Updates message status
     */
    int Receive(char* buffer);
    uint8_t Checksum(char* buffer, int size);
    int Respond(std::string data);
};

#endif /* _RSP_SERVER_H */
