#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME:=orca-generic
CONFIG:= -DCYCLES_TO_SIM=10000000 -DNOC_H_SIZE=4 -DNOC_W_SIZE=4

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
GCC_FLAGS  :=-I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include -Iinclude \
	-lpthread $(OPT_PLAT_FLAGS) $(COMPILER_FLAGS) $(SYSTEM_FLAGS)

LD_FLAGS   :=-L../../bin -lmod -lsim

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
	
	