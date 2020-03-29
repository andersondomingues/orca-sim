#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME:=orca-dma

#Path to dependencies. This platform is similulated using
#URSA, so libsim and libmod are required. 
LIBSIM_PATH:=../../simulator
LIBMOD_PATH:=../../models
OUTPUTDIR :=./bin
SOURCEDIR :=./src
HEADERDIR :=./include

LIBSIM:=$(LIBSIM_PATH)/bin/libsim.a

include ./Configuration.mk

#Flags to be used during compilation of the platform. Note 
#that they may be different form flags used for compiling
#dependencies.
CXXFLAGS  +=-I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include \
    -Iinclude -lpthread $(COMPLINE) $(GLOBAL_SETTINGS)

#added -Wl,--allow-multiple-definition for possible bug in binutils/ld
#as recommended in https://sourceware.org/bugzilla/show_bug.cgi?id=12762
LD_FLAGS   :=-L../../bin -lsim
#-Wl,--allow-multiple-definition

#source files for the platform
TARGET_DEPS := \
	$(SOURCEDIR)/Tile.cpp \
	$(SOURCEDIR)/ProcessingTile.cpp \
	$(SOURCEDIR)/NetworkTile.cpp \
	$(SOURCEDIR)/Orca.cpp \
	$(LIBMOD_PATH)/src/THellfireProcessor.cpp \
	$(LIBMOD_PATH)/src/UMemory.cpp \
	$(LIBMOD_PATH)/src/USignal.cpp \
	$(LIBMOD_PATH)/src/UBuffer.cpp \
	$(LIBMOD_PATH)/src/TNetBridge.cpp \
	$(LIBMOD_PATH)/src/TRouter.cpp \
	$(LIBMOD_PATH)/src/TDmaNetif.cpp

# include headers in the dependecy check
HEADER_FILES := $(wildcard $(HEADERDIR)/*.h)
HEADER_FILES += $(LIBMOD_PATH)/include/THellfireProcessor.h \
	$(LIBMOD_PATH)/include/UMemory.h \
	$(LIBMOD_PATH)/include/USignal.h \
	$(LIBMOD_PATH)/include/UBuffer.h \
	$(LIBMOD_PATH)/include/TNetBridge.h \
	$(LIBMOD_PATH)/include/TRouter.h \
	$(LIBMOD_PATH)/include/TDmaNetif.h

#compiler an instance of the simulator 
$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(TARGET_DEPS) $(HEADER_FILES)
	$(Q)g++ $(TARGET_DEPS) -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe $(LD_FLAGS) $(CXXFLAGS)

#remove previously generated files
clean:
	$(Q)rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME).exe
	
	
