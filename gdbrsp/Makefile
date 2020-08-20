#directory configuration
SOURCEDIR :=./src
HEADERDIR :=./include
OUTPUTDIR :=./bin
LIBNAME :=libgdbrsp.a

#include global configuration
include ../Configuration.mk

LIBSIM_PATH:=../ursa
LIBSIM:=$(LIBSIM_PATH)/bin/libursa.a

LIBMOD_PATH:=../models
LIBMOD:=$(LIBMOD_PATH)/bin/libmod.a

#include flags (plus external flags)
CPPFLAGS :=-Iinclude -I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include -c $(MAIN_COMPLINE)

#list of dependencies (source files)
TARGET_DEPS := \
	$(OUTPUTDIR)/UdpAsyncServer.o \
	$(OUTPUTDIR)/RspServer.o \
	$(OUTPUTDIR)/DataConvertionHelper.o 

#compress all generated object files into a pack
$(OUTPUTDIR)/$(LIBNAME): $(TARGET_DEPS)
	$(Q)ar rcs $(OUTPUTDIR)/$(LIBNAME) $(TARGET_DEPS)

#compile all classes (.cpp) into %.o files
$(OUTPUTDIR)/%.o: $(SOURCEDIR)/%.cpp  $(HEADERDIR)/%.hpp 
	$(Q)g++ $(CPPFLAGS) $< -o $@ 

#remove any preoviosly generated file
clean:
	$(Q)rm -rf bin/*.a rm -rf bin/*.o

