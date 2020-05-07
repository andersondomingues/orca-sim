#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME :=orca-mpsoc

#Path to dependencies. 
LIBSIM_PATH :=../../ursa
LIBMOD_PATH :=../../models
LIBRSP_PATH :=../../gdbrsp

OUTPUTDIR :=./bin
SOURCEDIR :=./src
HEADERDIR :=./include

LIBMOD:=$(LIBMOD_PATH)/bin/libmod.a
LIBSIM:=$(LIBSIM_PATH)/bin/libursa.a
LIBRSP:=$(LIBRSP_PATH)/bin/libgdbrsp.a

#include local config
include ./Configuration.mk

#include global config, required to avoid including gdbrsp
#into the compilation when the support is disabled
include ../../Configuration.mk

#Flags to be used during compilation of the platform. Note 
#that they may be different form flags used for compiling
#dependencies.
CXXFLAGS  +=-I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include \
    -I$(LIBRSP_PATH)/include -Iinclude \
    -lpthread $(MAIN_COMPLINE) $(PLAT_COMPLINE) $(MODELS_COMPLINE)

#link with other libraries
LD_FLAGS :=-L../../bin -lmod -lursa

# nullate gdblib when gdb support is absent
ifneq ($(ORCA_ENABLE_GDBRSP),YES)
	LIBRSP := $(LIBRSP_PATH)/bin/keep.me
else
	LD_FLAGS := $(LD_FLAGS) -lgdbrsp
endif

#source files for the platform
TARGET_DEPS := \
	$(SOURCEDIR)/Tile.cpp \
	$(SOURCEDIR)/ProcessingTile.cpp \
	$(SOURCEDIR)/NetworkTile.cpp \
	$(SOURCEDIR)/Orca.cpp

DEP_LIBS := $(LIBRSP) $(LIBMOD) $(LIBSIM)

# include headers in the dependecy check
HEADER_FILES := $(wildcard $(HEADERDIR)/*.h)

#compiler an instance of the simulator 
$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(TARGET_DEPS) $(HEADER_FILES) $(DEP_LIBS)
	$(Q)g++ $(TARGET_DEPS) -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe $(LD_FLAGS) $(CXXFLAGS)

#remove previously generated files
clean:
	$(Q)rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME).exe

