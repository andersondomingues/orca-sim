all:
	@echo "$'\e[036mSimulator... \e[0m"
	make -C simulator
	@echo "$'\e[036mModels... \e[0m"
	make -C models
	@echo "$'\e[036mSoftware... \e[0m"
	make -C software
	@echo "$'\e[036mPlatform... \e[0m"
	make -C platform
