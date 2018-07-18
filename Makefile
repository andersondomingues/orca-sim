all:
	@echo "$'\e[036mSimulator... \e[0m"
	make -C simulator
	@echo "$'\e[036mModels... \e[0m"
	make -C models
	@echo "$'\e[036mSoftware... \e[0m"
	make -C software
	@echo "$'\e[036mPlatform... \e[0m"
	make -C platform
	@echo "$'\e[036mSimulation Started! \e[0m"
	./platform/bin/teste ./software/bin/code.bin

clean:
	@echo "$'\e[036mSimulator... \e[0m"
	make -C simulator clean
	@echo "$'\e[036mModels... \e[0m"
	make -C models clean
	@echo "$'\e[036mSoftware... \e[0m"
	make -C software clean
	@echo "$'\e[036mPlatform... \e[0m"
	make -C platform clean


