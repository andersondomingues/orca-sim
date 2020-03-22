include ../../Configuration.mk

LIBNAME = liborca-client.a
SRCDIR  = ./src
BINDIR  = ./bin
INCDIR  = ./include

CPPFLAGS = -O3 -g -Wall -Wextra -Werror -I$(INCDIR) -c $(COMPLINE)

TRGTDEPS = \
	$(BINDIR)/orca-network-base.o \
	$(BINDIR)/orca-udp-client-server.o

#compile all classes (.cpp) into %.o files
$(BINDIR)/%.o: $(SRCDIR)/%.cpp 
	$(Q)g++ $(CPPFLAGS) $< -o $@ -c 

#make application executable
$(BINDIR)/$(LIBNAME): $(TRGTDEPS)
	$(Q)ar rcs $(BINDIR)/$(LIBNAME) $(TRGTDEPS)

all: $(BINDIR)/$(LIBNAME)
	$(Q)echo "done."

clean:
	$(Q)rm -rf ./bin/*.exe ./bin/*.o
