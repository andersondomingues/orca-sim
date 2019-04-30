LIBNAME = orca-lib.a
SRCDIR  = ./src
BINDIR  = ./bin
INCDIR  = ./include

CPPFLAGS = -O3 -g -Wall -Wextra -Werror -I$(INCDIR) -c
TRGTDEPS = \
	$(BINDIR)/udp_client_server.o \
	$(BINDIR)/orca-lib.o

#compile all classes (.cpp) into %.o files
$(BINDIR)/%.o: $(SRCDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ -c 

#make application executable
$(BINDIR)/$(LIBNAME): $(TRGTDEPS)
	ar rcs $(BINDIR)/$(LIBNAME) $(TRGTDEPS)

all: $(BINDIR)/$(LIBNAME)
	echo "done."

clean:
	rm -rf ./bin/*.exe ./bin/*.o