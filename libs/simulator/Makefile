SOURCEDIR :=./src
HEADERDIR :=./include
OUTPUTDIR :=./bin

CPP := g++
CPPFLAGS := -g -c -Og -I$(HEADERDIR) -Wall -Wextra -Werror 
#-fsanitize=address

ARFLAGS :=
LIBNAME := libsim.a 

all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "all done!"

#pack file into a static library to be used later
$(OUTPUTDIR)/$(LIBNAME): $(OUTPUTDIR)/Metric.o $(OUTPUTDIR)/Model.o $(OUTPUTDIR)/TimedModel.o $(OUTPUTDIR)/UntimedModel.o $(OUTPUTDIR)/Event.o $(OUTPUTDIR)/Simulator.o 
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o

#compile all classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp 
	$(CPP) $(CPPFLAGS) $< -o $@ 
	
.PHONY: clean
clean:
	rm -rf $(OUTPUTDIR)/*.o $(OUTPUTDIR)/*.a
