#includes local configuration
include Configuration.mk

#include global config (required by gdbrsp support)
include ../Configuration.mk

#directory configuration
SOURCEDIR :=src
HEADERDIR :=include
OUTPUTDIR :=bin

#libname
LIBNAME :=libmod.a

#requires libursa
LIBSIM_PATH:=../ursa
LIBSIM:=$(LIBSIM_PATH)/bin/libursa.a

#include flags (plus external flags)
CPPFLAGS :=-I./include -I../ursa/include -I../gdbrsp/include -c $(MAIN_COMPLINE) $(MODELS_COMPLINE) $(PLAT_COMPLINE)

#list of dependencies (source files)
TARGET_DEPS := \
	$(OUTPUTDIR)/TNetBridge.o \
	$(OUTPUTDIR)/USignal.o \
	$(OUTPUTDIR)/USignalSet.o \
	$(OUTPUTDIR)/UBuffer.o \
	$(OUTPUTDIR)/UMemory.o \
	$(OUTPUTDIR)/TRouter.o \
	$(OUTPUTDIR)/TProcessorBase.o \
	$(OUTPUTDIR)/THFRiscV.o \
	$(OUTPUTDIR)/TDmaNetif.o

#compress all generated object files into a pack
$(OUTPUTDIR)/$(LIBNAME): $(LIBSIM) $(TARGET_DEPS)
	$(Q)ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)

#compile all classes (.cpp) into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp  $(HEADERDIR)/%.h 
	$(Q)g++ $(CPPFLAGS) $< -o $@ 

#remove any preoviosly generated file
clean:
	$(Q)rm -rf bin/*.a 	rm -rf bin/*.o
