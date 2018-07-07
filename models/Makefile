SOURCEDIR :=./src
GCCFLAGS  := -I. -I../simulator
LDFLAGS   := 
OUTPUTDIR :=./bin

LIBNAME :=libmodels.a

all: $(OUTPUTDIR)/HFRiscv.o
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o
	cp $(OUTPUTDIR)/$(LIBNAME) ../bin/$(LIBNAME)

$(OUTPUTDIR)/HFRiscv.o: 
	gcc -c $(SOURCEDIR)/HFRiscv.cpp $(GCCFLAGS) -o $(OUTPUTDIR)/HFRiscv.o -l:libudes.a
