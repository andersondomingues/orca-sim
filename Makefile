SIMLIB:=simulator/bin/libsim.a
MODLIB:=models/bin/libmodels.a

SOFTBIN:=software/bin/code.bin
PLATEXE:=platform/bin/teste

all: $(SIMLIB) $(MODLIB) $(SOFTBIN) $(PLATEXE)
	@echo "$'\e[036mSimulation Started! \e[0m"
	./$(PLATEXE) $(SOFTBIN)

$(PLATEXE):
	@echo "$'\e[036mPlatform... \e[0m"
	make -C platforms

$(SOFTBIN):
	@echo "$'\e[036mBuilding Software BIN... \e[0m"
	make -C software

$(MODLIB):
	@echo "$'\e[036mBuilding Models LIB... \e[0m"
	make -C models

$(SIMLIB):
	@echo "$'\e[036mBuilding Simulator LIB... \e[0m"
	make -C simulator

clean:
	@echo "$'\e[036mSimulator... \e[0m"
	make -C simulator clean
	@echo "$'\e[036mModels... \e[0m"
	make -C models clean
	@echo "$'\e[036mSoftware... \e[0m"
	make -C software clean
	@echo "$'\e[036mPlatform... \e[0m"
	make -C platforms clean


