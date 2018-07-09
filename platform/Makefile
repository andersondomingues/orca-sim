GCCFLAGS  := -O3 -g -I../simulator -I../models 
LDFLAGS   := -L../models/bin -L../simulator/bin -lmodels -lsim
OUTPUTDIR :=./bin
LIBNAME := teste

LIBS:= ../models/bin/libmodels.a ../simulator/bin/libsim.a

all: teste
	$(OUTPUTDIR)/$(LIBNAME)
	
teste: main.cpp 
	g++ main.cpp $(GCCFLAGS) -o  $(OUTPUTDIR)/$(LIBNAME) $(LIBS)


