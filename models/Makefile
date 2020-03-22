#directory configuration
SOURCEDIR :=./src
OUTPUTDIR :=./bin
LIBNAME :=libmod.a

#include flags (plus external flags)
CPPFLAGS :=-I./include -I../simulator/include -c $(COMPLINE) $(GLOBAL_SETTINGS)

#list of dependencies (source files)
TARGET_DEPS := \
	$(OUTPUTDIR)/TNetBridge.o \
	$(OUTPUTDIR)/USignal.o \
	$(OUTPUTDIR)/UBuffer.o \
	$(OUTPUTDIR)/UMemory.o \
	$(OUTPUTDIR)/TRouter.o \
	$(OUTPUTDIR)/THellfireProcessor.o \
	$(OUTPUTDIR)/TDmaNetif.o


#$(OUTPUTDIR)/TDmaNetif.o
#$(OUTPUTDIR)/TArm7TDMI.o

#compress all generated object files into a pack
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)

#compile all classes (.cpp) into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp 
	g++ $(CPPFLAGS) $< -o $@ 

#remove any preoviosly generated file
clean:
	rm -rf bin/*.a 	rm -rf bin/*.o
