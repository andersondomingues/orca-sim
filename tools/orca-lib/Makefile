APPNAME = orca-lib
SRCDIR  = ./src
BINDIR  = ./bin
INCDIR  = ./include

CPPFLAGS = -O3 -g -Wall -Wextra -Werror -I$(INCDIR) -c
TRGTDEPS = \
	$(BINDIR)/udp_client_server.o \
	$(BINDIR)/libhf.o

#compile all classes (.cpp) into %.o files
$(BINDIR)/%.o: $(SRCDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ -c 

#make application executable
all: $(TRGTDEPS)
	ar rcs $(BINDIR)/liborca.a $(TRGTDEPS)

clear:
	rm -rf ./bin/*.exe ./bin/*.o