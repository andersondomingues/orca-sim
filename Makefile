#configuration
PLATFORM     := orca-generic
APPLICATION  := noc_test4
export APPLICATION

#libnames
PLATFORM_BIN      := orca-generic.exe
SIMULATOR_LIB     := libsim.a
MODELS_LIB        := libmod.a
APPLICATION_IMAGE := code0.bin

#include optmizations
include Configuration.mk

#directory configurations (paths)
SIMULATOR_DIR := $(CURDIR)/simulator
BINARY_DIR    := $(CURDIR)/bin
PLATFORMS_DIR := $(CURDIR)/platforms
MODELS_DIR    := $(CURDIR)/models
TOOLS_DIR     := $(CURDIR)/tools
APP_DIR       := $(CURDIR)/software

#compile everything if necessary and run
#simulatotion requires the simulator and software 
# - software has no dependencies
# - simulator has no dependencies
# - hardware models depends on the simulator
# - platform depends on simulator and hardware models
all: $(BINARY_DIR)/$(PLATFORM_BIN) $(BINARY_DIR)/$(APPLICATION_IMAGE)
	@echo "$'\e[7m====================================\e[0m"
	@echo "$'\e[7m  All done! Starting simulation...  \e[0m"
	@echo "$'\e[7m====================================\e[0m"
	@echo $(OPTMIZATION_FLAGS)
	$(BINARY_DIR)/$(PLATFORM_BIN) $(BINARY_DIR)/$(APPLICATION_IMAGE) 

#URSA's simulation library
$(BINARY_DIR)/$(SIMULATOR_LIB):
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m     Building URSA's libsim       \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	make -C $(SIMULATOR_DIR)
	cp $(SIMULATOR_DIR)/bin/$(SIMULATOR_LIB) $(BINARY_DIR)/$(SIMULATOR_LIB)

#library containing hardware models
$(BINARY_DIR)/$(MODELS_LIB): $(BINARY_DIR)/$(SIMULATOR_LIB)
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m     Building hardware models     \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	make -C $(MODELS_DIR)
	cp $(MODELS_DIR)/bin/$(MODELS_LIB) $(BINARY_DIR)/$(MODELS_LIB)

#platform executable
$(BINARY_DIR)/$(PLATFORM_BIN): $(BINARY_DIR)/$(SIMULATOR_LIB) $(BINARY_DIR)/$(MODELS_LIB)
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m     Building the platform        \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	make -C $(PLATFORMS_DIR)/$(PLATFORM)
	cp $(PLATFORMS_DIR)/$(PLATFORM)/bin/$(PLATFORM_BIN) $(BINARY_DIR)/$(PLATFORM_BIN)

#software (kernel + application)
$(BINARY_DIR)/$(APPLICATION_IMAGE):
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m Building software (kernel + apps)\e[0m"
	@echo "$'\e[7m==================================\e[0m"
	make -C $(APP_DIR) images

clean:
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m          Cleaning up...          \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	@make -C $(SIMULATOR_DIR) clean
	@make -C $(MODELS_DIR) clean
	@make -C $(PLATFORMS_DIR)/$(PLATFORM) clean
	@make -C $(APP_DIR) clean	
	#@rm -rf $(BINARY_DIR)/*
