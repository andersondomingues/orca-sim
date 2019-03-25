APPNAME = orca-app-loader
SRCDIR  = ./src
BINDIR  = ./bin
INCDIR  = ./include

CPPFLAGS = -O3 -g -Wall -Wextra -Werror -lpthread -I$(INCDIR)
TRGTDEPS = \
	$(BINDIR)/udp_client_server.o \
	$(BINDIR)/console.o \
	$(BINDIR)/libhf.o

#compile all classes (.cpp) into %.o files
$(BINDIR)/%.o: $(SRCDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ -c 

#make application executable
all: $(TRGTDEPS)
	g++ $(BINDIR)/*.o -o $(BINDIR)/$(APPNAME).exe $(CPPFLAGS)
	$(BINDIR)/$(APPNAME).exe 2 ./bin/noc_test4_sender.bin noc_test4_sender

clear:
	rm -rf ./bin/*.exe ./bin/*.o