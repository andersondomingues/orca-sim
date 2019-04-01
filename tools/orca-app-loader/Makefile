APPNAME = orca-app-loader
SRCDIR  = ./src
BINDIR  = ./bin
INCS    = -I./include -I../orca-lib/include

CPPFLAGS = -O3 -g -Wall -Wextra -Werror -lpthread $(INCS)
TRGTDEPS = \
	$(BINDIR)/app-loader.o

#compile all classes (.cpp) into %.o files
$(BINDIR)/%.o: $(SRCDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ -c 

#make application executable
all: $(TRGTDEPS)
	make -C ../orca-lib
	cp ../orca-lib/bin/liborca.a $(BINDIR)
	g++ $(BINDIR)/*.o $(BINDIR)/liborca.a -o $(BINDIR)/$(APPNAME).exe $(CPPFLAGS)
	$(BINDIR)/$(APPNAME).exe 5 ../../bin/noc_test4_sender.bin noc_test4_sender 127.0.0.1 9999

clean:
	rm -rf ./bin/*.exe ./bin/*.o
	