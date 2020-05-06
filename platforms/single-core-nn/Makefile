#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME:=single-core-nn

#Path to dependencies. This platform is similulated using
#URSA, so libsim and libmod are required. 
LIBSIM_PATH:=../../simulator
LIBMOD_PATH:=../../models
LIBRSP_PATH:=../../gdbrsp/
OUTPUTDIR :=./bin
SOURCEDIR :=./src
HEADERDIR :=./include

LIBSIM:=../../bin/libsim.a
LIBRSP:=../../bin/libgdbrsp.a


#Flags to be used during compilation of the platform. Note 
#that they may be different form flags used for compiling
#dependencies.
CXXFLAGS  :=-I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include \
    -I$(LIBRSP_PATH)/include -Iinclude -lpthread $(COMPLINE) $(GLOBAL_SETTINGS)

#added -Wl,--allow-multiple-definition for possible bug in binutils/ld
#as recommended in https://sourceware.org/bugzilla/show_bug.cgi?id=12762
LD_FLAGS   :=-L../../bin -lsim -lgdbrsp
#-Wl,--allow-multiple-definition

#source files for the platform
TARGET_DEPS := \
	$(SOURCEDIR)/ProcessingTile.cpp \
	$(SOURCEDIR)/TDmaMult.cpp \
	$(SOURCEDIR)/Orca.cpp \
	$(LIBMOD_PATH)/src/TProcessorBase.cpp \
	$(LIBMOD_PATH)/src/THFRiscV.cpp \
	$(LIBMOD_PATH)/src/UMemory.cpp \
	$(LIBMOD_PATH)/src/USignal.cpp

# include headers in the dependecy check
HEADER_FILES := $(wildcard $(HEADERDIR)/*.h)
HEADER_FILES += $(LIBMOD_PATH)/include/THFRiscV.h \
	$(LIBMOD_PATH)/include/TProcessorBase.h \
	$(LIBMOD_PATH)/include/UMemory.h \
	$(LIBMOD_PATH)/include/USignal.h


#compiler an instance of the simulator 
$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(LIBRSP) $(LIBSIM) $(TARGET_DEPS) $(HEADER_FILES)
	$(Q)g++ $(CXXFLAGS) $(TARGET_DEPS) $(LD_FLAGS) -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe 

#remove previously generated files
clean:
	$(Q)rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME).exe

