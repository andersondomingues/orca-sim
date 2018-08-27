#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME:=ModelTest-HFRiscv32

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

#Path to the operating system. HellfireOS provide some scripts for compiling
#a 3x3 noc platform.
HELLFIREOS_PATH:=../../submodules/hellfireos/platform/noc_3x3
HELLFIREOS_ARCH:=../../submodules/hellfireos/arch/riscv/hf-riscv
#This is the master recipie. It compiles all the stuff and launchs the simulation.
all: $(LIBSIM) $(LIBMOD) $(HELLFIREOS_PATH)/code0.bin $(OUTPUTDIR)/$(PLATFORM_NAME)
	@echo "$'\e[032mStarting... \e[0m"
	$(OUTPUTDIR)/$(PLATFORM_NAME) ./bin/code0.bin

#Recipies for compiling dependencies.
$(LIBSIM):
	@echo "$'\e[036mBuilding simulation library... \e[0m"
	@make -C $(LIBSIM_PATH)

$(LIBMOD):
	@echo "$'\e[036mBuilding models library... \e[0m"
	@make -C $(LIBMOD_PATH)

$(HELLFIREOS_PATH)/code0.bin:
	@echo "$'\e[036mPatching HellfireOS... \e[0m"
	cp hellfire-patch/ni.c $(HELLFIREOS_ARCH)/drivers/ni.c
	cp hellfire-patch/ni.h $(HELLFIREOS_ARCH)/include/ni.h
	cp hellfire-patch/arch.mak $(HELLFIREOS_ARCH)/arch.mak
	cp hellfire-patch/makefile $(HELLFIREOS_PATH)/makefile
	@echo "$'\e[036mBuilding HellfireOS... \e[0m"
	@make -C $(HELLFIREOS_PATH) images -j8
	cp $(HELLFIREOS_PATH)/*.bin ./bin

$(OUTPUTDIR)/$(PLATFORM_NAME): $(SOURCEDIR)/platform.cpp
	@echo "$'\e[036mBuilding target platform ($(PLATFORM_NAME))... \e[0m"
	g++ $(SOURCEDIR)/platform.cpp $(GCCFLAGS) $(LDFLAGS) -o $(OUTPUTDIR)/$(PLATFORM_NAME)

#The old, gold cleanup
clean:
	@echo "$'\e[033mCleaning simulation library... \e[0m"
	make -C $(LIBSIM_PATH) clean
	@echo "$'\e[033mCleaning models library... \e[0m"
	make -C $(LIBMOD_PATH) clean
	@echo "$'\e[033mCleaning HellfireOS... \e[0m"
	make -C $(HELLFIREOS_PATH) clean
	@echo "$'\e[033mCleaning target platform $(PLATFORM_NAME)... \e[0m"
	rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME)
	rm -rf $(OUTPUTDIR)/code*


