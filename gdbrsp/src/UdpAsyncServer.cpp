#include <UdpAsyncServer.h>
#include <iostream>

using namespace std;

void UdpAsyncServer::Error(UdpAsyncError err){
    
    std::string message = "UdpAsyncErro: ";

    switch(err){
        case UdpAsyncError::SOCKET_OPEN:
            message = message + "Could not create socket (tips: +exceeded_socket_limit +system_permission).";
            break;
        case UdpAsyncError::SOCKET_BIND:
            message = message + "Could not bind socket (tips: +address_in_use).";
    }

    std::cout << message << std::endl;

}

UdpAsyncServer::UdpAsyncServer(int port){

    if ((socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1){
        this->Error(UdpAsyncError::SOCKET_OPEN);
    }

    int flags = fcntl(socket_fd, F_GETFL);
    flags |= O_NONBLOCK;
    fcntl(socket_fd, F_SETFL, flags);
    
    FD_ZERO(&original_socket);
    FD_ZERO(&original_stdin);
    FD_ZERO(&readfds);
    FD_ZERO(&writefds);
    
    FD_SET(socket_fd, &original_socket);//instead of 0 put socket_fd
    FD_SET(socket_fd, &readfds);
    FD_SET(0,&original_stdin);
    FD_SET(0, &writefds);
    
    // since we got s2 second, it's the "greater", so we use that for
    // // the n param in select()
    numfd = socket_fd + 1;
    
    // wait until either socket has data ready to be recv()d (timeout 10.5 secs)
    tv.tv_sec = 10;
    tv.tv_usec = 500000;
    
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(SERVER_PORT);
    server_address.sin_addr.s_addr = INADDR_ANY;
    bzero(&(server_address.sin_zero),8);
    
    if (bind(socket_fd,(struct sockaddr *)&server_address, sizeof(struct sockaddr)) == -1
    ) {
         this->Error(UdpAsyncError::SOCKET_BIND);
    }

    address_length = sizeof(struct sockaddr);
    std::cout << "GDBRSP up at UDP port " << port << std::endl;
 
}


int UdpAsyncServer::Recv(char* data){
   
        readfds = original_socket;
        writefds = original_stdin;//problem

        int res = select(numfd, &readfds, &writefds, NULL, &tv);

        switch(res){

            case -1:
                this->Error(UdpAsyncError::SELECT_ERR);
                break;
            case 0:
                this->Error(UdpAsyncError::TIMEOUT_EXCEEDED);
                break;
            default:
                if (FD_ISSET(socket_fd, &readfds)) { 
                    
                    FD_CLR(socket_fd, &readfds);
                    bytes_read = recvfrom(socket_fd, data, MAX_LENGTH, 0,
                        (struct sockaddr *)&client_address, &address_length);
                    return bytes_read;    
                }
        }
        
        return 0;
}


UdpAsyncServer::~UdpAsyncServer(){
    close(socket_fd);
}


int main(){

    char buffer[4000];

    UdpAsyncServer* server = new UdpAsyncServer(5000);

    while(1){
        server->Recv(buffer);
    }


}
