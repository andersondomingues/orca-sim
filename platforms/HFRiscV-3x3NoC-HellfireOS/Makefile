PROJ_NAME :=HFRiscV-3x3NoC-HellfireOS


GCCFLAGS  := -g -I../../simulator/include -I../../models/include -Wall -Werror -O3 
LDFLAGS   := -L../../models/bin -L../../simulator/bin -L../../generics/bin -lmod -lsim 
OUTPUTDIR :=./bin
SOURCEDIR :=./src

LIBNAME := $(PROJ_NAME)

DEPENDENCIES:= ../../models/bin/libmod.a ../../simulator/bin/libsim.a 

all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "all done!"
	
$(OUTPUTDIR)/$(LIBNAME): 
	g++ $(SOURCEDIR)/test_platform.cpp $(GCCFLAGS) -o  $(OUTPUTDIR)/$(LIBNAME) $(DEPENDENCIES) $(LDFLAGS)

clean:
	rm -rf $(OUTPUTDIR)/$(LIBNAME)
	
