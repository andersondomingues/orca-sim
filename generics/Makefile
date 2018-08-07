SOURCEDIR :=./src
HEADERDIR :=./
OUTPUTDIR :=./bin

CPP := g++
CPPFLAGS := -g -c -O3 -I. -Wall
ARFLAGS :=
LIBNAME := libgen.a 

all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "done."

$(OUTPUTDIR)/$(LIBNAME): $(OUTPUTDIR)/Buffer.o $(OUTPUTDIR)/Memory.o 
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o

#compile all classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp 
	$(CPP) $(CPPFLAGS) $< -o $@ 
	
clean:
	rm -rf $(OUTPUTDIR)/*.o $(OUTPUTDIR)/*.a
