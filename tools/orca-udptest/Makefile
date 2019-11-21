LIBORCA = liborca-client.a
APPNAME1 = orca-udptest-client
APPNAME2 = orca-udptest-server
SRCDIR  = ./src
BINDIR  = ./bin
INCS    = -I./include -I../orca-lib-client/include

CPPFLAGS = -O3 -g -Wall -Wextra -Werror $(INCS)
TRGTDEPS = \
	$(BINDIR)/orca-udptest-client.o \
	$(BINDIR)/orca-udptest-server.o

#compile all classes (.cpp) into %.o files
$(BINDIR)/%.o: $(SRCDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ -c 

#make application executable
all: $(TRGTDEPS) 
	make -C ../orca-lib-client
	cp ../orca-lib-client/bin/$(LIBORCA) $(BINDIR)
	make -C ../orca-lib-client clean
	g++ $(BINDIR)/orca-udptest-client.o $(BINDIR)/$(LIBORCA) -o $(BINDIR)/$(APPNAME1).exe $(CPPFLAGS)
	g++ $(BINDIR)/orca-udptest-server.o $(BINDIR)/$(LIBORCA) -o $(BINDIR)/$(APPNAME2).exe $(CPPFLAGS)
	mv $(BINDIR)/$(APPNAME1).exe ../../bin
	mv $(BINDIR)/$(APPNAME2).exe ../../bin

clean:
	rm -rf ./bin/*.exe ./bin/*.o ./bin/*.a
	
