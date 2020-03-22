LIBORCA = orca-lib.a
APPNAME = orca-remote-control
SRCDIR  = ./src
BINDIR  = ./bin
INCS    = -I./include -I../orca-lib-client/include

CPPFLAGS = -O3 -g -Wall -Wextra -Werror $(INCS)
TRGTDEPS = \
	$(BINDIR)/orca-remote.o

#compile all classes (.cpp) into %.o files
$(BINDIR)/%.o: $(SRCDIR)/%.cpp 
	$(Q)g++ $(CPPFLAGS) $< -o $@ -c 

#make application executable
all: $(TRGTDEPS)
	$(Q)make -C ../orca-lib-client
	$(Q)cp ../orca-lib-client/bin/$(LIBORCA) $(BINDIR)
	$(Q)g++ $(BINDIR)/*.o $(BINDIR)/$(LIBORCA) -o $(BINDIR)/$(APPNAME).exe $(CPPFLAGS)

clean:
	$(Q)rm -rf ./bin/*.exe ./bin/*.o
	
