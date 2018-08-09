#needed by "module load" to work
#SHELL:=/bin/bash

APPNAME :=single_core
#APPNAME := noc_3x3 images

SIMLIB  :=simulator/bin/libsim.a
GENLIB  :=generics/bin/libgen.a
MODLIB  :=models/bin/libmod.a
SOFTBIN :=software/bin/code.bin
PLATEXE :=platforms/bin/teste
OSIMG   :=hellfireos/platform/$(APPNAME)/image.bin
#OSIMG   :=./code.bin

#---------------------------- RUN
#6th) When all pieces are compiled, we run our platform.
all: $(PLATEXE) $(OSIMG)
	@echo "$'\e[036mSimulating...\e[0m"
	./$(PLATEXE) ./$(OSIMG)
	@echo "$'\e[036mFinished! \e[0m"

#--------------------------- SIMULATOR AND HARWARE
#1st) Compile simulation lib. 
#Simulation lib contains classes for instanting the 
#simulation engine.
$(SIMLIB): 
	@echo "$'\e[036mBuilding Simulator Library... \e[0m"
	make -C simulator

#2nd) Compile generics lib. Depends on SIMLIB.
#Generics lib contains hardware often used within platforms,
#such as memory and buffers.
$(GENLIB): $(SIMLIB)
	@echo "$'\e[036mBuilding Generics Library... \e[0m"
	make -C generics

#3rd) compile models lib.
#Models are the harware to be simulated. Note that it only
#contains self-contained modules, not the platforms.
$(MODLIB): $(GENLIB)
	@echo "$'\e[036mBuilding Models Library... \e[0m"
	make -C models

#4th) Lastly, we build the platform itself
$(PLATEXE): $(SIMLIB) $(MODLIB) $(GENLIB)
	@echo "$'\e[036mPlatform... \e[0m"
	make -C platforms
#---------------------------- SOFTWARE
#5th) OPTIONAL: build an operating system. Maybe the platform
#does not require software. In this case, skip this step. In this
#example we use hellfireOS.
$(OSIMG):
	@echo "$'\e[036mBuilding HellfireOS Image... \e[0m"
	make -C hellfireos/platform/$(APPNAME) image

#------- cleanup
clean:
	@echo "$'\e[036mCleaning Simulator... \e[0m"
	make -C simulator clean
	@echo "$'\e[036mCleaning Generics... \e[0m"
	make -C generics clean
	@echo "$'\e[036mCleaning Models... \e[0m"
	make -C models clean
	@echo "$'\e[036mCleaning Hellfire... \e[0m"
	make -C hellfireos/platform/$(APPNAME) clean
	@echo "$'\e[036mCleaning Platform... \e[0m"
	make -C platforms clean


