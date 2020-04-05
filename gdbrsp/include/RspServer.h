#ifndef _RSP_SERVER_H
#define _RSP_SERVER_H

#include <UdpServer.h>
#include <RspPCore.h>

#define RSP_BUFFER_SIZE 1000


class RspServer{

private:
    //upd server instance, see UdpServe.h
    UdpServer* _server;

    //ip address and udp port for THIS processor
    std::string _ipaddr;
    uint32_t _udpport;

    //buffers to communicate with gdbcliet
    char _buffer_in[RSP_BUFFER_SIZE];
    char _buffer_out[RSP_BUFFER_SIZE];

    //pcore reference
    RspPCore* _pcore;

public:
    /**
     * Ctor.
     */
    RspServer(std::string ipaddr, uint32_t udpport, RspPCore* core);

    /**
     * Dtor.
     */
    ~RspServer();
   
    /**
     * Updates message status
     */
    RspUpdate();
};

#endif /* _RSP_SERVER_H */
