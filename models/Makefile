SOURCEDIR :=./src
OUTPUTDIR :=./bin

CPP:=g++

CPPFLAGS  :=-I. -I../simulator -I../generics -c -g -O3 -Wall

LIBNAME   :=libmod.a

all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "done."

$(OUTPUTDIR)/$(LIBNAME): $(OUTPUTDIR)/HFRiscvModel.o $(OUTPUTDIR)/MemoryModel.o $(OUTPUTDIR)/DmniModel.o $(OUTPUTDIR)/NocRouterModel.o
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o

#compile all classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp 
	$(CPP) $(CPPFLAGS) $< -o $@ 

clean:
	rm -rf bin/*.a 	rm -rf bin/*.o
