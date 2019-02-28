#This is the name of the platform. It is used only 
#to name the generated executable file and to be
#displayed during execution of this makefile
PLATFORM_NAME:=Sulphane
CONFIG:= -DCYCLES_TO_SIM=10000000 -DNOC_H_SIZE=4 -DNOC_W_SIZE=4

#Path to dependencies. This platform is similulated using
#URSA, so libsim and libmod are required. 
LIBSIM_PATH:=../../simulator
LIBMOD_PATH:=../../models

LIBMOD:=$(LIBMOD_PATH)/bin/libmod.a
LIBSIM:=$(LIBSIM_PATH)/bin/libsim.a

#Flags to be used during compilation of the platform. Note 
#that they may be different form flags used for compiling
#dependencies.
GCCFLAGS  := -g -I$(LIBSIM_PATH)/include -I$(LIBMOD_PATH)/include -Wextra -Wall -Werror -O3 -lpthread
#-fsanitize=address

LDFLAGS   := -L$(LIBSIM_PATH)/bin -L$(LIBMOD_PATH)/bin -lmod -lsim 
OUTPUTDIR :=./bin
SOURCEDIR :=./src

#Path to the operating system. 
HELLFIREOS_PATH:=../../software/hellfireos/platform/ursa
HELLFIREOS_ARCH:=../../software/hellfireos/arch/riscv/hf-riscv

#This is the master recipie. It compiles all the stuff and launchs the simulation.
all: $(LIBSIM) $(LIBMOD) $(OUTPUTDIR)/$(PLATFORM_NAME).exe
	@echo "$'\e[032m==SULPHANE==\e[0m"
	$(OUTPUTDIR)/$(PLATFORM_NAME).exe ./bin/

#Recipies for compiling dependencies.
$(LIBSIM):
	@echo "$'\e[036mBuilding simulation library... \e[0m"
	@make -C $(LIBSIM_PATH)

$(LIBMOD):
	@echo "$'\e[036mBuilding models library... \e[0m"
	@make -C $(LIBMOD_PATH)

$(HELLFIREOS_PATH)/code0.bin:
	@echo "$'\e[036mBuilding HellfireOS... \e[0m"
	@make -C $(HELLFIREOS_PATH) images
	cp $(HELLFIREOS_PATH)/*.bin ./bin

$(OUTPUTDIR)/$(PLATFORM_NAME).exe: $(SOURCEDIR)/Tile.cpp $(SOURCEDIR)/ProcessingTile.cpp $(SOURCEDIR)/NetworkTile.cpp $(SOURCEDIR)/Sulphane.cpp 
	@echo "$'\e[036mBuilding target platform ($(PLATFORM_NAME))... \e[0m"
	g++ $(CONFIG) $(SOURCEDIR)/Tile.cpp $(SOURCEDIR)/NetworkTile.cpp $(SOURCEDIR)/ProcessingTile.cpp $(SOURCEDIR)/Sulphane.cpp $(GCCFLAGS) $(LDFLAGS) -o $(OUTPUTDIR)/$(PLATFORM_NAME).exe -Iinclude 

app:
	@echo "$'\e[033mCleaning HellfireOS... \e[0m"
	make -C $(HELLFIREOS_PATH) clean
	rm -rf $(OUTPUTDIR)/code*
	@echo "$'\e[036mBuilding HellfireOS... \e[0m"
	@make -C $(HELLFIREOS_PATH) images
	cp $(HELLFIREOS_PATH)/*.bin ./bin

#The old, gold cleanup
clean:
	@echo "$'\e[033mCleaning simulation library... \e[0m"
	make -C $(LIBSIM_PATH) clean
	@echo "$'\e[033mCleaning models library... \e[0m"
	make -C $(LIBMOD_PATH) clean
	@echo "$'\e[033mCleaning target platform $(PLATFORM_NAME)... \e[0m"
	rm -rf $(OUTPUTDIR)/$(PLATFORM_NAME).exe

#@echo "$'\e[033mCleaning logs... \e[0m"
#rm -rf logs/*.log
	
	


