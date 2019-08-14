/** 
 * This file is part of project URSA. More information on the project
 * can be found at URSA's repository at GitHub
 * 
 * http://https://github.com/andersondomingues/ursa
 *
 * Copyright (C) 2018 Anderson Domingues, <ti.andersondomingues@gmail.com>
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
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA. **/

//std API
#include <iostream>
#include <sstream>
#include <chrono>

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>

#include <TNetSocket.h>


//udp bullshit
#include <stdio.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/**
 * @brief Default ctor. Name of the module must be informed.
 * @param name A name to identify this module.
 */
TNetSocket::TNetSocket(std::string name) : TimedModel(name) {

	//open debug file
	output_debug.open(
		"logs/pe-0-0.cpu_debug.log", 
		std::ofstream::out | std::ofstream::trunc);
	
	//create a new comm so that the module can be interrupted by
	//the external network when a new packet arrives
	_comm_recv = new UComm<int8_t>(&_recv_val, 0, this->GetName() + ".commUdpIntr");
	_comm_recv->Write(0);
	
	//reset traffic counter
	_trafficOut = 0;
	_trafficIn  = 0;
	
	//initialize states
	_recv_state = TNetSocketRecvState::READY;
	_send_state = TNetSocketSendState::WAIT;
	
	//initialize a new client (client sends messages)
	const std::string& client_addr = NETSOCKET_CLIENT_ADDRESS;
	_udp_client = new udp_client(client_addr, NETSOCKET_CLIENT_PORT);

	const std::string& server_addr = NETSOCKET_SERVER_ADDRESS;
	_udp_server = new udp_server(server_addr, NETSOCKET_SERVER_PORT);
		
	pthread_t t;

	if(pthread_create(&t, NULL, TNetSocket::udpRecvThread, this)){
		std::cout << "unable to create new thread using lpthread." << std:: endl;
	}
	
	//this code depends on linux's libraries. I warned you.
	output_debug << "UDP bridge is up" << std::endl;
	
	this->Reset();
}

void TNetSocket::SetCommAck(UComm<int8_t>* p){ _comm_ack = p; }
void TNetSocket::SetCommIntr(UComm<int8_t>* p){ _comm_intr = p; }
void TNetSocket::SetCommStart(UComm<int8_t>* p){ _comm_start = p; }
void TNetSocket::SetCommStatus(UComm<int8_t>* p){ _comm_status = p; }

UComm<int8_t>* TNetSocket::GetCommRecv(){
	return _comm_recv;
}

void TNetSocket::SetMem1(UMemory* mem1){
	_mem1 = mem1;
}
void TNetSocket::SetMem2(UMemory* mem2){
	_mem2 = mem2;
}

uint8_t* TNetSocket::GetBuffer(){
	return _recv_buffer;
}

/**
 * @brief Dtor. No dynamic allocation is being used. Keept by design.
 */
TNetSocket::~TNetSocket(){
	output_debug.close();
}

/**
 * @brief Return the module to its initial state (if stateful).
 */
void TNetSocket::Reset(){
	
}


udp_server* TNetSocket::GetUdpServer(){
	return _udp_server;
}

/**
 * @brief Runs a state.
 * @return The number of cycles spent to change (or not) states.
 */
long long unsigned int TNetSocket::Run(){
    
	this->udpToNocProcess(); //process for receiving from the UDP socket
    this->nocToUdpProcess(); //process for sending through the UDP socket
    return 1; //takes exactly 1 cycle to run both processes
}

void* TNetSocket::udpRecvThread(void* gs){
	
	TNetSocket* ns = ( TNetSocket*) gs;

	//recv while the program lives
	while(1){
		
		//recv only when interface is not busy
		if(ns->GetCommRecv()->Read() == 0x0){
			
			//recvs from external network
			ns->GetUdpServer()->recv((char*)(ns->GetBuffer()), RECV_BUFFER_LEN);
			
			//start interface
			ns->GetCommRecv()->Write(0x1);
		}	
	}
}

void TNetSocket::LogWrite(std::string ss){
	this->output_debug << ss << std::flush;
}

void TNetSocket::udpToNocProcess(){
	
	switch(_recv_state){
		
		//waiting for new packages
		case TNetSocketRecvState::READY:{
			
			//packet has arrived and the network is not sending packets
			if(_comm_recv->Read() == 0x1 && _comm_start->Read() == 0x0){
				
				//copy buffer to internal memory
				_mem2->Write(_mem2->GetBase(), (int8_t*)_recv_buffer, RECV_BUFFER_LEN);
				
				#ifndef OPT_DISABLE_LOG_NETSOCKET
				uint16_t addr_flit;
				_mem2->Read(_mem2->GetBase(), (int8_t*)&addr_flit, 2);
				
				int32_t x = (addr_flit & 0xf0) >> 4;
				int32_t y = (addr_flit & 0x0f);
				int32_t z = x + y * 4;
				
				std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();				
				auto duration = now.time_since_epoch();
				auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
				
				output_debug << "[" << millis << "] IN " << _trafficIn
							 << " FROM " << _udp_server->get_addr() << ":"	 << _udp_server->get_port() 
							 << " TO #" << z << std::endl << std::flush;
				#endif

				//start bursting packets into the noc
				_recv_state = TNetSocketRecvState::DATA_IN;
				_trafficIn++;
			}
			
		}break;
		
		case TNetSocketRecvState::DATA_IN:{
			
			//able to send
			if(_comm_start->Read() == 0){
				
				_comm_start->Write(0x1); //enable netif
				
				//start bursting packets into the noc
				_recv_state = TNetSocketRecvState::FLUSH;
			}
			
		}break;
		
		case TNetSocketRecvState::FLUSH:{
			
			//netif finished, we can receive another packet
			if(_comm_start->Read() == 0){
				_recv_state = TNetSocketRecvState::READY;
				_comm_recv->Write(0x0);
			}
			
		}break;
	}	
}

void TNetSocket::nocToUdpProcess(){	
    
	switch(_send_state){
		
		//wait until has some packet to send
		case TNetSocketSendState::WAIT:{
			
			if(_comm_intr->Read() == 0x1){ 
						
				//64 flits = 1 msg
				int8_t msg[RECV_BUFFER_LEN];
				
				//get packet from the scratchpad
				_mem1->Read(_mem1->GetBase(), msg, RECV_BUFFER_LEN);

				//send packet (raw) via udp
				_udp_client->send((const char*)msg, RECV_BUFFER_LEN);
						
				#ifndef OPT_NETSOCKET_DISABLE_OUTGOING_PACKETS_LOG
				uint32_t x = msg[4];
				
				std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();				
				auto duration = now.time_since_epoch();
				auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
				
				output_debug << "[" << millis << "] OUT " << _trafficOut
							 << " TO " << _udp_client->get_addr() << ":" << _udp_client->get_port() 
							 << " FROM #" << x << std::endl << std::flush;
				#endif
				
				//signal the ni that the packet has been consumed
				_comm_ack->Write(0x01);
				_send_state = TNetSocketSendState::WAIT_FOR_ACK;
				//std::cout << "to WAIT_FOR_ACK" << std::endl;
				
				//delete[] msg; //free tmp buffer
			}
			
		} break;
		
		//and wait for ack-ack
		case TNetSocketSendState::WAIT_FOR_ACK:{
			
			if(_comm_intr->Read() == 0x0){
				_send_state = TNetSocketSendState::LOWER_ACK;
				//std::cout << "to WAIT" << std::endl;				
			}
				
		} break;
		
		//lower ack signal
		case TNetSocketSendState::LOWER_ACK:{
			
			//lower ack
			_comm_ack->Write(0x00);
			_trafficOut++;
			_send_state = TNetSocketSendState::WAIT;
			//std::cout << "to WAIT_NAK" << std::endl;				
			
		} break;
		
	}
}


//----------------------- UDP stuff
//reference: https://linux.m2osw.com/c-implementation-udp-clientserver


// ========================= CLIENT =========================

/** \brief Initialize a UDP client object.
 *
 * This function initializes the UDP client object using the address and the
 * port as specified.
 *
 * The port is expected to be a host side port number (i.e. 59200).
 *
 * The \p addr parameter is a textual address. It may be an IPv4 or IPv6
 * address and it can represent a host name or an address defined with
 * just numbers. If the address cannot be resolved then an error occurs
 * and constructor throws.
 *
 * \note
 * The socket is open in this process. If you fork() or exec() then the
 * socket will be closed by the operating system.
 *
 * \warning
 * We only make use of the first address found by getaddrinfo(). All
 * the other addresses are ignored.
 *
 * \exception udp_client_server_runtime_error
 * The server could not be initialized properly. Either the address cannot be
 * resolved, the port is incompatible or not available, or the socket could
 * not be created.
 *
 * \param[in] addr  The address to convert to a numeric IP.
 * \param[in] port  The port number.
 */
udp_client::udp_client(const std::string& addr, int port)
    : f_port(port)
    , f_addr(addr)
{
    char decimal_port[16];
    snprintf(decimal_port, sizeof(decimal_port), "%d", f_port);
    decimal_port[sizeof(decimal_port) / sizeof(decimal_port[0]) - 1] = '\0';
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    int r(getaddrinfo(addr.c_str(), decimal_port, &hints, &f_addrinfo));
    if(r != 0 || f_addrinfo == NULL)
    {
        throw udp_client_server_runtime_error(("invalid address or port: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    f_socket = socket(f_addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    if(f_socket == -1)
    {
        freeaddrinfo(f_addrinfo);
        throw udp_client_server_runtime_error(("could not create socket for: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
}

/** \brief Clean up the UDP client object.
 *
 * This function frees the address information structure and close the socket
 * before returning.
 */
udp_client::~udp_client()
{
    freeaddrinfo(f_addrinfo);
    close(f_socket);
}

/** \brief Retrieve a copy of the socket identifier.
 *
 * This function return the socket identifier as returned by the socket()
 * function. This can be used to change some flags.
 *
 * \return The socket used by this UDP client.
 */
int udp_client::get_socket() const
{
    return f_socket;
}

/** \brief Retrieve the port used by this UDP client.
 *
 * This function returns the port used by this UDP client. The port is
 * defined as an integer, host side.
 *
 * \return The port as expected in a host integer.
 */
int udp_client::get_port() const
{
    return f_port;
}

/** \brief Retrieve a copy of the address.
 *
 * This function returns a copy of the address as it was specified in the
 * constructor. This does not return a canonalized version of the address.
 *
 * The address cannot be modified. If you need to send data on a different
 * address, create a new UDP client.
 *
 * \return A string with a copy of the constructor input address.
 */
std::string udp_client::get_addr() const
{
    return f_addr;
}

/** \brief Send a message through this UDP client.
 *
 * This function sends \p msg through the UDP client socket. The function
 * cannot be used to change the destination as it was defined when creating
 * the udp_client object.
 *
 * The size must be small enough for the message to fit. In most cases we
 * use these in Snap! to send very small signals (i.e. 4 bytes commands.)
 * Any data we would want to share remains in the Cassandra database so
 * that way we can avoid losing it because of a UDP message.
 *
 * \param[in] msg  The message to send.
 * \param[in] size  The number of bytes representing this message.
 *
 * \return -1 if an error occurs, otherwise the number of bytes sent. errno
 * is set accordingly on error.
 */
int udp_client::send(const char *msg, size_t size)
{
    return sendto(f_socket, msg, size, 0, f_addrinfo->ai_addr, f_addrinfo->ai_addrlen);
}



// ========================= SEVER =========================

/** \brief Initialize a UDP server object.
 *
 * This function initializes a UDP server object making it ready to
 * receive messages.
 *
 * The server address and port are specified in the constructor so
 * if you need to receive messages from several different addresses
 * and/or port, you'll have to create a server for each.
 *
 * The address is a string and it can represent an IPv4 or IPv6
 * address.
 *
 * Note that this function calls connect() to connect the socket
 * to the specified address. To accept data on different UDP addresses
 * and ports, multiple UDP servers must be created.
 *
 * \note
 * The socket is open in this process. If you fork() or exec() then the
 * socket will be closed by the operating system.
 *
 * \warning
 * We only make use of the first address found by getaddrinfo(). All
 * the other addresses are ignored.
 *
 * \exception udp_client_server_runtime_error
 * The udp_client_server_runtime_error exception is raised when the address
 * and port combinaison cannot be resolved or if the socket cannot be
 * opened.
 *
 * \param[in] addr  The address we receive on.
 * \param[in] port  The port we receive from.
 */
udp_server::udp_server(const std::string& addr, int port)
    : f_port(port)
    , f_addr(addr)
{
    char decimal_port[16];
    snprintf(decimal_port, sizeof(decimal_port), "%d", f_port);
    decimal_port[sizeof(decimal_port) / sizeof(decimal_port[0]) - 1] = '\0';
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    int r(getaddrinfo(addr.c_str(), decimal_port, &hints, &f_addrinfo));
    if(r != 0 || f_addrinfo == NULL)
    {
        throw udp_client_server_runtime_error(("invalid address or port for UDP socket: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    f_socket = socket(f_addrinfo->ai_family, SOCK_DGRAM | SOCK_CLOEXEC, IPPROTO_UDP);
    if(f_socket == -1)
    {
        freeaddrinfo(f_addrinfo);
        throw udp_client_server_runtime_error(("could not create UDP socket for: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
    r = bind(f_socket, f_addrinfo->ai_addr, f_addrinfo->ai_addrlen);
    if(r != 0)
    {
        freeaddrinfo(f_addrinfo);
        close(f_socket);
        throw udp_client_server_runtime_error(("could not bind UDP socket with: \"" + addr + ":" + decimal_port + "\"").c_str());
    }
}

/** \brief Clean up the UDP server.
 *
 * This function frees the address info structures and close the socket.
 */
udp_server::~udp_server()
{
    freeaddrinfo(f_addrinfo);
    close(f_socket);
}

/** \brief The socket used by this UDP server.
 *
 * This function returns the socket identifier. It can be useful if you are
 * doing a select() on many sockets.
 *
 * \return The socket of this UDP server.
 */
int udp_server::get_socket() const
{
    return f_socket;
}

/** \brief The port used by this UDP server.
 *
 * This function returns the port attached to the UDP server. It is a copy
 * of the port specified in the constructor.
 *
 * \return The port of the UDP server.
 */
int udp_server::get_port() const
{
    return f_port;
}

/** \brief Return the address of this UDP server.
 *
 * This function returns a verbatim copy of the address as passed to the
 * constructor of the UDP server (i.e. it does not return the canonalized
 * version of the address.)
 *
 * \return The address as passed to the constructor.
 */
std::string udp_server::get_addr() const
{
    return f_addr;
}

/** \brief Wait on a message.
 *
 * This function waits until a message is received on this UDP server.
 * There are no means to return from this function except by receiving
 * a message. Remember that UDP does not have a connect state so whether
 * another process quits does not change the status of this UDP server
 * and thus it continues to wait forever.
 *
 * Note that you may change the type of socket by making it non-blocking
 * (use the get_socket() to retrieve the socket identifier) in which
 * case this function will not block if no message is available. Instead
 * it returns immediately.
 *
 * \param[in] msg  The buffer where the message is saved.
 * \param[in] max_size  The maximum size the message (i.e. size of the \p msg buffer.)
 *
 * \return The number of bytes read or -1 if an error occurs.
 */
int udp_server::recv(char *msg, size_t max_size)
{
    return ::recv(f_socket, msg, max_size, 0);
}

/** \brief Wait for data to come in.
 *
 * This function waits for a given amount of time for data to come in. If
 * no data comes in after max_wait_ms, the function returns with -1 and
 * errno set to EAGAIN.
 *
 * The socket is expected to be a blocking socket (the default,) although
 * it is possible to setup the socket as non-blocking if necessary for
 * some other reason.
 *
 * This function blocks for a maximum amount of time as defined by
 * max_wait_ms. It may return sooner with an error or a message.
 *
 * \param[in] msg  The buffer where the message will be saved.
 * \param[in] max_size  The size of the \p msg buffer in bytes.
 * \param[in] max_wait_ms  The maximum number of milliseconds to wait for a message.
 *
 * \return -1 if an error occurs or the function timed out, the number of bytes received otherwise.
 */
int udp_server::timed_recv(char *msg, size_t max_size, int max_wait_ms)
{
    fd_set s;
    FD_ZERO(&s);
    FD_SET(f_socket, &s);
    struct timeval timeout;
    timeout.tv_sec = max_wait_ms / 1000;
    timeout.tv_usec = (max_wait_ms % 1000) * 1000;
  
    /**
     * 23/03/2019: Anderson
     * Removed last two parameters to avoid -Werror=restrict 
     * original call:
     * 		=> int retval = select(f_socket + 1, &s, &s, &s, &timeout);
     * TODO: make sure that these parameters won't break the function
     */
    int retval = select(f_socket + 1, &s, 0, 0, &timeout);
    if(retval == -1)
    {
        // select() set errno accordingly
        return -1;
    }
    if(retval > 0)
    {
        // our socket has data
        return ::recv(f_socket, msg, max_size, 0);
    }

    // our socket has no data
    errno = EAGAIN;
    return -1;
}
