#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME:=single-core

#Path to dependencies. 
LIBSIM_PATH :=../../simulator
LIBMOD_PATH :=../../models
LIBRSP_PATH :=../../gdbrsp

OUTPUTDIR :=./bin
SOURCEDIR :=./src
HEADERDIR :=./include

LIBMOD:=$(LIBMOD_PATH)/bin/libmod.a
LIBSIM:=$(LIBSIM_PATH)/bin/libsim.a
LIBRSP:=$(LIBRSP_PATH)/bin/libgdbrsp.a

#Flags to be used during compilation of the platform. Note 
#that they may be different form flags used for compiling  
#dependencies.
CXXFLAGS  +=-I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include \
    -I$(LIBRSP_PATH)/include -Iinclude -lpthread $(COMPLINE) $(GLOBAL_SETTINGS)

#added -Wl,--allow-multiple-definition for possible bug in binutils/ld
#as recommended in https://sourceware.org/bugzilla/show_bug.cgi?id=12762
LD_FLAGS   :=-L../../bin -lmod -lsim -lgdbrsp
#-Wl,--allow-multiple-definition

#source files for the platform
TARGET_DEPS := \
	$(SOURCEDIR)/ProcessingTile.cpp \
	$(SOURCEDIR)/Orca.cpp

DEP_LIBS := $(LIBRSP) $(LIBMOD) $(LIBSIM)

# include headers in the dependecy check
HEADER_FILES := $(wildcard $(HEADERDIR)/*.h)

#compiler an instance of the simulator 
$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(TARGET_DEPS) $(HEADER_FILES) $(DEP_LIBS)
	$(Q)g++ $(TARGET_DEPS) -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe $(LD_FLAGS) $(GCC_FLAGS)

#remove previously generated files
clean:
	$(Q)rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME).exe

