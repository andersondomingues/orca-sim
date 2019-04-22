#directory configuration
SOURCEDIR :=./src
OUTPUTDIR :=./bin
LIBNAME :=libmod.a

#include flags (plus external flags)
CPPFLAGS :=-I./include -I../simulator/include -c $(COMPLINE)

#list of dependencies (source files)
TARGET_DEPS := \
	$(OUTPUTDIR)/TNetSocket.o \
	$(OUTPUTDIR)/UComm.o \
	$(OUTPUTDIR)/UBuffer.o \
	$(OUTPUTDIR)/UMemory.o \
	$(OUTPUTDIR)/TNetif.o \
	$(OUTPUTDIR)/TRouter.o \
	$(OUTPUTDIR)/THellfireProcessor.o

#compress all generated object files into a pack
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(OUTPUTDIR)/*o

#compile all classes (.cpp) into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ 

#remove any preoviosly generated file
clean:
	rm -rf bin/*.a 	rm -rf bin/*.o
