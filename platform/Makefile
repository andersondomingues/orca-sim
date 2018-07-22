GCCFLAGS  := -g -I../simulator -I../models -Wall -O3 
LDFLAGS   := -L../models/bin -L../simulator/bin -lmodels -lsim
OUTPUTDIR :=./bin
LIBNAME := teste

#APPNAME :=sort.bin
APPNAME :=code.bin

LIBS:= ../models/bin/libmodels.a ../simulator/bin/libsim.a

all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "done."
	
$(OUTPUTDIR)/$(LIBNAME): main.cpp
	g++ main.cpp $(GCCFLAGS) -o  $(OUTPUTDIR)/$(LIBNAME) $(LIBS)

clean:
	rm -rf $(OUTPUTDIR)/$(LIBNAME) 
