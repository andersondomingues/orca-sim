GCCFLAGS  := -g -I../simulator -I../models -I../generics -Wall -O3 
LDFLAGS   := -L../models/bin -L../simulator/bin -L../generics/bin -lmodels -lsim -lgen
OUTPUTDIR :=./bin
LIBNAME := teste

#platform to compile

#PLATFORM :=main.cpp
#PLATFORM :=example_noc3x3.cpp
#PLATFORM :=example_dmni.cpp
PLATFORM :=3x3-FULL-MPSOC.cpp

LIBS:= ../models/bin/libmod.a ../simulator/bin/libsim.a ../generics/bin/libgen.a

all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "done."
	
$(OUTPUTDIR)/$(LIBNAME): $(PLATFORM)
	g++ $(PLATFORM) $(GCCFLAGS) -o  $(OUTPUTDIR)/$(LIBNAME) $(LIBS)

clean:
	rm -rf $(OUTPUTDIR)/$(LIBNAME) 

	
