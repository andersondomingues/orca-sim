SOURCEDIR :=./src
OUTPUTDIR :=./bin

CPP:=g++

#some useful flags:
#"-DNOGUARDS" disables all IFs (not kidding, all guards down!)
CPPFLAGS :=-I./include -I../simulator/include -c -g -O3 -Wall -DNOGUARDS 



LIBNAME :=libmod.a


TARGET_DEPS := $(OUTPUTDIR)/UBuffer.o $(OUTPUTDIR)/UMemory.o $(OUTPUTDIR)/TDmni.o $(OUTPUTDIR)/TRouter.o $(OUTPUTDIR)/THellfireProcessor.o


all: $(OUTPUTDIR)/$(LIBNAME)
	@echo "all done!"

$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o

#compile all classes into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp 
	$(CPP) $(CPPFLAGS) $< -o $@ 

clean:
	rm -rf bin/*.a 	rm -rf bin/*.o
