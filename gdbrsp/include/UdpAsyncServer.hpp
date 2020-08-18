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
#ifndef GDBRSP_INCLUDE_UDPASYNCSERVER_HPP_
#define GDBRSP_INCLUDE_UDPASYNCSERVER_HPP_

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/select.h>  // use select() for multiplexing
#include <sys/fcntl.h>  // for non-blocking

#define MAX_LENGTH 1024
#define SERVER_PORT 5000

enum class UdpAsyncError{
    SOCKET_OPEN,  // socket could not be opened, see fd limit or permission
    SOCKET_BIND,  // socket could not bind, see if port is in use
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
    char* recieve_data[MAX_LENGTH], send_data[MAX_LENGTH];
    struct sockaddr_in server_address , client_address;

 public:
    /**
     * Default ctor.
     * @param port UDP port number to run the server.
     */
    explicit UdpAsyncServer(int port);

    ~UdpAsyncServer();

    /**
     * Send a reply to the GDB client.
     * @param data Data to be send
     * @param length The number of bytes to send
     * @return Returns 0 when sucefful, otherwise error num.
     */
    int Send(char* data, int length);

    /**
     * Receives data from the GDB client.
     * @param data Location to store data in.
     * @return Returns the number of received bytes or -1
     * */
    int Receive(char* data);

    void Error(UdpAsyncError err);
};

#endif  // GDBRSP_INCLUDE_UDPASYNCSERVER_HPP_
