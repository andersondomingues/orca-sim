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

//simulator API
#include <TimedModel.h>
#include <UBuffer.h>

#include <TNetSocket.h>

//bullshit from the system
#include <unistd.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
  
#define PORT	8080 
#define MAXLINE 1024 

/**
 * @brief Default ctor. Name of the module must be informed.
 * @param name A name to identify this module.
 */
TNetSocket::TNetSocket(std::string name) : TimedModel(name) {
	/*
	//this code depends on linux's libraries. I warned you.
	std::cout << this->GetName() << " (warning): this module is unable to run in Windows machines." << std::endl;
	
	//initialize sending socket
    if ((_send_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        
		stringstream ss;
		ss << this->GetName() << ": unable to create socket for sending messages"
		   << "through UDP network." << std::endl;
		   
		throw runtime_error(ss.str());
    } 
    
	//initialize sending address
    //memset(&servaddr, 0, sizeof(servaddr)); 
    //memset(&cliaddr, 0, sizeof(cliaddr)); 
      
    // Filling server information 
    servaddr.sin_family    = AF_INET; // IPv4 
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT); 
	
	cliaddr = servaddr;
	  
    // Bind the socket with the server address 
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0 ){ 
        perror("bind failed"); 
    } 
	
	
	
	
	this->Reset();*/
}

/**
 * @brief Dtor. No dynamic allocation is being used. Keept by design.
 */
TNetSocket::~TNetSocket(){}

/**
 * @brief Return the module to its initial state (if stateful).
 */
void TNetSocket::Reset(){}

/**
 * @brief Runs a state.
 * @return The number of cycles spent to change (or not) states.
 */
long long unsigned int TNetSocket::Run(){
    
	//this->recvProcess(); //process for receiving from the UDP socket
    //this->sendProcess(); //process for sending through the UDP socket
    return 1; //takes exactly 1 cycle to run both processes
}

void TNetSocket::recvProcess(){
	
	
	
}


void TNetSocket::sendProcess(){	

    /*int len, n; 
	
    n = recvfrom(sockfd, (char *)buffer, MAXLINE,  
                MSG_WAITALL, ( struct sockaddr *) &cliaddr, 
                &len); 
				
    buffer[n] = '\0'; 
    printf("Client : %s\n", buffer); */
	/*
    sendto(sockfd, hello.c_str(), sizeof(hello.c_str()),  
        MSG_CONFIRM, (const struct sockaddr *) &cliaddr, 
            sizeof(hello)); 
    printf("Hello message sent.\n");  */
	
}
