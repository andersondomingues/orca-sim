LIBORCA = orca-usched.a
APPNAME = orca-usched
SRCDIR  = ./src
BINDIR  = ./bin
INCS    = -I./include

CPPFLAGS = -O3 -g -Wall -Wextra -Werror $(INCS)  

GTK_FLAGS_P = `pkg-config --cflags gtk+-3.0`
GTK_LINKK_P = `pkg-config --libs gtk+-3.0`

TRGTDEPS = \
	$(BINDIR)/orca-usched.o

#compile all classes (.cpp) into %.o files
$(BINDIR)/%.o: $(SRCDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ -c $(GTK_FLAGS_P)

#make application executable
all: $(TRGTDEPS)
	g++ $(BINDIR)/*.o -o $(BINDIR)/$(APPNAME).exe $(CPPFLAGS) $(GTK_FLAGS_P) $(GTK_LINKK_P)

clean:
	rm -rf ./bin/*.exe ./bin/*.o
	
