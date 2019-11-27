LIBORCA = orca-usched.a
APPNAME = orca-usched
SRCDIR  = ./src
BINDIR  = ./bin
INCS    = -I./include -I/usr/include/gtk-3.0

CPPFLAGS = -O3 -g -Wall -Wextra -Werror $(INCS) -pthread 
TRGTDEPS = \
	$(BINDIR)/orca-usched.o

#compile all classes (.cpp) into %.o files
$(BINDIR)/%.o: $(SRCDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ -c 

#make application executable
all: $(TRGTDEPS)
	g++ $(BINDIR)/*.o $(BINDIR)/$(LIBORCA) -o $(BINDIR)/$(APPNAME).exe $(CPPFLAGS)

clean:
	rm -rf ./bin/*.exe ./bin/*.o
	
