SOURCEDIR :=./src
OUTPUTDIR :=./bin

GCCFLAGS  :=-I. -I../simulator
LDFLAGS   :=-l:libudes.a

LIBNAME   :=libmodels.a


all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "done."

$(OUTPUTDIR)/$(LIBNAME): $(OUTPUTDIR)/HFRiscv.o $(OUTPUTDIR)/MemoryHelper.o
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o

$(OUTPUTDIR)/HFRiscv.o: $(SOURCEDIR)/HFRiscv.cpp HFRiscv.h
	gcc -c $(SOURCEDIR)/HFRiscv.cpp $(GCCFLAGS) -o $(OUTPUTDIR)/HFRiscv.o $(LDFLAGS)

$(OUTPUTDIR)/MemoryHelper.o: $(SOURCEDIR)/MemoryHelper.cpp MemoryHelper.h
	gcc -c $(SOURCEDIR)/MemoryHelper.cpp $(GCCFLAGS) -o $(OUTPUTDIR)/MemoryHelper.o $(LDFLAGS)

clean:
	rm -rf bin/*.a 	rm -rf bin/*.o