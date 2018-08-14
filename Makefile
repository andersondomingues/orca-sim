current_platform :=HFRiscV-3x3NoC-HellfireOS

imgpath := ./hellfireos/platform/noc_3x3/

libsim :=./simulator/bin/libsim.a
libmod :=./models/bin/libmod.a
plaexe :=./platforms/$(current_platform)/bin/$(current_platform)
osimg  :=$(imgpath)/code0.bin




#---------------- master recipie
all: $(libsim) $(libmod) $(osimg) $(plaexe)
	@echo "$'\e[032mStarting... \e[0m"
	./platforms/$(current_platform)/bin/$(current_platform) $(imgpath)

#---------------- dependencies
$(libsim):
	@echo "$'\e[036mBuilding simulation library... \e[0m"
	@make -C simulator

$(libmod):
	@echo "$'\e[036mBuilding models library... \e[0m"
	@make -C models

$(plaexe):
	@echo "$'\e[036mBuilding target platform ($(current_platform))... \e[0m"
	@make -C platforms/$(current_platform)

$(osimg):
	@echo "$'\e[036mBuilding operating system... \e[0m"
	@make -C $(imgpath) images

#---------------- cleanup
clean:
	@echo "$'\e[033mCleaning simulation library... \e[0m"
	@make -C simulator clean
	@echo "$'\e[033mCleaning models library... \e[0m"
	@make -C models clean
	@echo "$'\e[033mCleaning platforms... \e[0m"
	@make -C platforms/$(current_platform) clean
