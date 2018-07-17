SOURCEDIR :=./src
HEADERDIR :=./
OUTPUTDIR :=./bin

#ar rcs sim-lib.a Process.o Event.o Simulator.o Buffer.o

CPPFLAGS := -g -c -O3 -I.
ARFLAGS :=
LIBNAME := libsim.a 

all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "done."

$(OUTPUTDIR)/$(LIBNAME): $(OUTPUTDIR)/Buffer.o $(OUTPUTDIR)/Event.o $(OUTPUTDIR)/Process.o $(OUTPUTDIR)/Simulator.o
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o

#compile all classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ 
	
clean:
	rm -rf $(OUTPUTDIR)/*
