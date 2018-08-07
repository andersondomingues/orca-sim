APPNAME :=single_core

SIMLIB  :=simulator/bin/libsim.a
MODLIB  :=models/bin/libmodels.a
SOFTBIN :=software/bin/code.bin
PLATEXE :=platforms/bin/teste
OS      :=software/bin/code.bin

all: $(SIMLIB) $(MODLIB) $(SOFTBIN) $(PLATEXE) $(OS)
	@echo "$'\e[036mSimulation Started! \e[0m"
	./$(PLATEXE) ./$(OS)

$(PLATEXE):
	@echo "$'\e[036mPlatform... \e[0m"
	make -C platforms

$(OS):
	@echo "$'\e[036mBuilding HellfireOS BIN... \e[0m"
	make -C hellfireos/platform/$(APPNAME) image
	mv ./hellfireos/platform/$(APPNAME)/image.bin $(OS)

$(MODLIB):
	@echo "$'\e[036mBuilding Models LIB... \e[0m"
	make -C models

$(SIMLIB):
	@echo "$'\e[036mBuilding Simulator LIB... \e[0m"
	make -C simulator

clean:
	@echo "$'\e[036mCleaning Simulator... \e[0m"
	make -C simulator clean
	@echo "$'\e[036mCleaning Models... \e[0m"
	make -C models clean
	@echo "$'\e[036mCleaning Hellfire... \e[0m"
	make -C hellfireos/platform/$(APPNAME) clean	
	rm -rf software/bin/code.bin
	@echo "$'\e[036mCleaning Platform... \e[0m"
	make -C platforms clean


