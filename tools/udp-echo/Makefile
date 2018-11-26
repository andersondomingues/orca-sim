
.PHONY: server client

server:
	g++ udp_client_server.cpp server.cpp -o server.exe
	./server.exe

client:
	g++ udp_client_server.cpp client.cpp -o client.exe
	./client.exe
	
clear:
	rm -rf *.exe *.o