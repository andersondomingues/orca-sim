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
	$(Q)g++ $(CPPFLAGS) $< -o $@ -c 

#make application executable
all: $(TRGTDEPS) 
	$(Q)make -C ../orca-lib-client
	$(Q)cp ../orca-lib-client/bin/$(LIBORCA) $(BINDIR)
	$(Q)make -C ../orca-lib-client clean
	$(Q)g++ $(BINDIR)/orca-udptest-client.o $(BINDIR)/$(LIBORCA) -o $(BINDIR)/$(APPNAME1).exe $(CPPFLAGS)
	$(Q)g++ $(BINDIR)/orca-udptest-server.o $(BINDIR)/$(LIBORCA) -o $(BINDIR)/$(APPNAME2).exe $(CPPFLAGS)
	$(Q)mv $(BINDIR)/$(APPNAME1).exe ../../bin
	$(Q)mv $(BINDIR)/$(APPNAME2).exe ../../bin

clean:
	$(Q)rm -rf ./bin/*.exe ./bin/*.o ./bin/*.a
	
