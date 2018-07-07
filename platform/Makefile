GCCFLAGS  := -O3 -g -I../simulator -I../models 
LDFLAGS   := -L../bin -lmodels -lsim
OUTPUTDIR :=./bin
LIBNAME := teste

all: teste
	mv $(LIBNAME) ../$(OUTPUTDIR)/$(LIBNAME)

teste: main.cpp 
	g++ main.cpp $(GCCFLAGS) -o $(LIBNAME) ../$(OUTPUTDIR)/*.a

