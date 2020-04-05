#ifndef _UDP_ASYNC_SERVER_H
#define _UDP_ASYNC_SERVER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/select.h>//use select() for multiplexing
#include <sys/fcntl.h> // for non-blocking

#define MAX_LENGTH 1024
#define SERVER_PORT 5000

using namespace std;

enum class UdpAsyncError{
    SOCKET_OPEN, //socket could not be opened, see fd limit or permission
    SOCKET_BIND, //socket could not bind, see if port is in use
    SELECT_ERR,
    TIMEOUT_EXCEEDED
};

/**
 * This class implements an asynchonous udp server. We use 
 * select() to interrupt the simulation only when there is 
 * some packet for the gdb server.
 * */
class UdpAsyncServer{

    private:
        fd_set original_socket;
        fd_set original_stdin;
        fd_set readfds;
        fd_set writefds;
    
        struct timeval tv;
        int numfd;
        int socket_fd, bytes_read;
        unsigned int address_length;
        char* recieve_data[MAX_LENGTH],send_data[MAX_LENGTH];
        struct sockaddr_in server_address , client_address;

    public:

        /**
         * Default ctor.
         * @param port UDP port number to run the server.
         */
        UdpAsyncServer(int port);

        ~UdpAsyncServer();
        
        
        /**
         * Send a reply to the GDB client.
         * @param data Data to be send
         * @param length The number of bytes to send
         * @return Returns 0 when sucefful, otherwise error num.
         */
        int Send(char* data, uint32_t length);

        /**
         * Receives data from the GDB client.
         * @param data Location to store data in.
         * @return Returns the number of received bytes or -1
         * */
        int Recv(char* data);

        void Error(UdpAsyncError erra);

};


#endif /* _UDP_ASYNC_SERVER_H */
