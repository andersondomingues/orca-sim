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

LIBMOD:=$(LIBMOD_PATH)/bin/libmod.a
LIBSIM:=$(LIBSIM_PATH)/bin/libsim.a

#Flags to be used during compilation of the platform. Note 
#that they may be different form flags used for compiling
#dependencies.
GCC_FLAGS  :=-I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include \
    -Iinclude -lpthread $(COMPLINE) $(GLOBAL_SETTINGS)

#added -Wl,--allow-multiple-definition for possible bug in binutils/ld
#as recommended in https://sourceware.org/bugzilla/show_bug.cgi?id=12762
LD_FLAGS   :=-L../../bin -lmod -lsim
#-Wl,--allow-multiple-definition

#source files for the platform
TARGET_DEPS := \
	$(SOURCEDIR)/Tile.cpp \
	$(SOURCEDIR)/ProcessingTile.cpp \
	$(SOURCEDIR)/NetworkTile.cpp \
	$(SOURCEDIR)/Orca.cpp

#compiler an instance of the simulator 
$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(TARGET_DEPS)
	g++ $(TARGET_DEPS) -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe $(LD_FLAGS) $(GCC_FLAGS)

#remove previously generated files
clean:
	rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME).exe
	
	
