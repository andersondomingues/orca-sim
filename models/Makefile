SOURCEDIR :=./src
OUTPUTDIR :=./bin

CPP:=g++
CPPFLAGS :=-I./include -I../simulator/include -c -g -O3 -Wall

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
