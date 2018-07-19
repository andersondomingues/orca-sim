SOURCEDIR :=./src
OUTPUTDIR :=./bin

GCCFLAGS  :=-I. -I../simulator
LDFLAGS   :=-l:libudes.a

LIBNAME   :=libmodels.a


all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "done."

$(OUTPUTDIR)/$(LIBNAME): $(OUTPUTDIR)/HFRiscv.o $(OUTPUTDIR)/MemoryModel.o $(OUTPUTDIR)/DmniModel.o
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o

$(OUTPUTDIR)/HFRiscv.o: $(SOURCEDIR)/HFRiscv.cpp HFRiscv.h
	gcc -c $(SOURCEDIR)/HFRiscv.cpp $(GCCFLAGS) -o $(OUTPUTDIR)/HFRiscv.o $(LDFLAGS)

$(OUTPUTDIR)/MemoryModel.o: $(SOURCEDIR)/MemoryModel.cpp MemoryModel.h
	gcc -c $(SOURCEDIR)/MemoryModel.cpp $(GCCFLAGS) -o $(OUTPUTDIR)/MemoryModel.o $(LDFLAGS)
	
$(OUTPUTDIR)/DmniModel.o: $(SOURCEDIR)/DmniModel.cpp DmniModel.h
	gcc -c $(SOURCEDIR)/DmniModel.cpp $(GCCFLAGS) -o $(OUTPUTDIR)/DmniModel.o $(LDFLAGS)

clean:
	rm -rf bin/*.a 	rm -rf bin/*.o
