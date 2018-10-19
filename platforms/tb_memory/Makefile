#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME:=MemoryTest

#Path to dependencies. This platform is similulated using
#URSA, so libsim and libmod are required. 
LIBSIM_PATH:=../../libs/simulator
LIBMOD_PATH:=../../libs/models

LIBMOD:=$(LIBMOD_PATH)/bin/libmod.a
LIBSIM:=$(LIBSIM_PATH)/bin/libsim.a

#Flags to be used during compilation of the platform. Note 
#that they may be different form flags used for compiling
#dependencies.
GCCFLAGS  := -g -I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include -Wall -Werror -O3 
LDFLAGS   := -L$(LIBSIM_PATH)/bin -L$(LIBMOD_PATH)/bin -lmod -lsim 
OUTPUTDIR :=./bin
SOURCEDIR :=./src

all: $(LIBSIM) $(LIBMOD) $(OUTPUTDIR)/$(PLATFORM_NAME).exe
	$(OUTPUTDIR)/$(PLATFORM_NAME).exe
	@echo "Done."

#Recipies for compiling dependencies.
$(LIBSIM):
	@echo "$'\e[036mBuilding simulation library... \e[0m"
	@make -C $(LIBSIM_PATH)

$(LIBMOD):
	@echo "$'\e[036mBuilding models library... \e[0m"
	@make -C $(LIBMOD_PATH)

$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(SOURCEDIR)/platform.cpp
	@echo "$'\e[036mBuilding target platform ($(PLATFORM_NAME))... \e[0m"
	g++ $(SOURCEDIR)/platform.cpp $(GCCFLAGS) $(LDFLAGS) -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe

#The old, gold cleanup
clean:
	@echo "$'\e[033mCleaning simulation library... \e[0m"
	make -C $(LIBSIM_PATH) clean
	@echo "$'\e[033mCleaning models library... \e[0m"
	make -C $(LIBMOD_PATH) clean
	@echo "$'\e[033mCleaning platform... \e[0m"
	rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME).exe


