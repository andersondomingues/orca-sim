
.PHONY: server client

server:
	g++ udp_client_server.cpp server.cpp -o server
	./server

client:
	g++ udp_client_server.cpp client.cpp -o client
	./client
	
clear:
	rm -rf *.out *.o