# Includes parameters from the configuration file
include ./Configuration.mk

# Name of URSA library, compiled as a static library.
URSA_LIB      := libsim.a

# Name of the library containing the hardware models. 
MODELS_LIB    := libmod.a

# name of the library containg the GDB RSP implementation.
GDBRSP_LIB    := libgdbrsp.a

# Additional parameters (do not modify them unless you know
# what you are doing here).
PLATFORM_BIN  := $(ORCA_PLATFORM).exe

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
export Q := @
# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory
endif

#directory configurations (paths)
URSA_DIR      := $(CURDIR)/simulator
BINARY_DIR    := $(CURDIR)/bin
PLATFORMS_DIR := $(CURDIR)/platforms
MODELS_DIR    := $(CURDIR)/models
GDBRSP_DIR    := $(CURDIR)/gdbrsp

#phonies (see https://www.gnu.org/software/make/manual/html_node/Phony-Targets.html)
.PHONY: clean documentation multitail tools

#compile everything if necessary and run
#simulatotion requires the simulator and software 
# - simulator has no dependencies
# - rsp server has no dependencies
# - hardware models depends on the simulator 
#   hardware models may depend on rsp
# - platform depends on simulato, hardware models, and rsp
# - visualization file for multitail has no dependencies
all: $(BINARY_DIR)/$(PLATFORM_BIN) vismtail
	@echo "$'\033[7m====================================\033[0m"
	@echo "$'\033[7m  All done!                         \033[0m"
	@echo "$'\033[7m====================================\033[0m"
	@echo " => $(PLATFORM_BIN) deployed to /bin folder."

#Generate the simulation library (uses URSA distro)
$(BINARY_DIR)/$(URSA_LIB): $(URSA_DIR)/src/*.cpp  $(URSA_DIR)/include/*.h 
	@echo "$'\033[7m==================================\033[0m"
	@echo "$'\033[7m     Building URSA's libsim       \033[0m"
	@echo "$'\033[7m==================================\033[0m"
	$(Q)make -C $(URSA_DIR) -j 8
	$(Q)cp $(URSA_DIR)/bin/$(URSA_LIB) $(BINARY_DIR)/$(URSA_LIB)

#Generate the library containing the hardware models (except for top-level hardware)
$(BINARY_DIR)/$(MODELS_LIB): $(BINARY_DIR)/$(URSA_LIB) $(MODELS_DIR)/src/*.cpp $(MODELS_DIR)/include/*.h
	@echo "$'\033[7m==================================\033[0m"
	@echo "$'\033[7m     Building hardware models     \033[0m"
	@echo "$'\033[7m==================================\033[0m"
	$(Q)make -C $(MODELS_DIR) -j 8
	$(Q)cp $(MODELS_DIR)/bin/$(MODELS_LIB) $(BINARY_DIR)/$(MODELS_LIB)

#Generate the GDB remote protocol library (used in processor core models). 
$(BINARY_DIR)/$(GDBRSP_LIB): $(GDBRSP_DIR)/src/*.cpp $(GDBRSP_DIR)/include/*.h
	@echo "$'\033[7m==================================\033[0m"
	@echo "$'\033[7m   Building GDB RSV support lib.  \033[0m"
	@echo "$'\033[7m==================================\033[0m"
	$(Q)make -C $(GDBRSP_DIR) -j 8
	$(Q)cp $(GDBRSP_DIR)/bin/$(GDBRSP_LIB) $(BINARY_DIR)/$(GDBRSP_LIB)

#Generate simulator executable binary (includes top-level hardware models).
$(BINARY_DIR)/$(PLATFORM_BIN): $(BINARY_DIR)/$(URSA_LIB) $(BINARY_DIR)/$(GDBRSP_LIB)  $(BINARY_DIR)/$(MODELS_LIB) $(PLATFORMS_DIR)/$(ORCA_PLATFORM)/src/*.cpp  $(PLATFORMS_DIR)/$(ORCA_PLATFORM)/include/*.h
	@echo "$'\033[7m==================================\033[0m"
	@echo "$'\033[7m     Building the platform        \033[0m"
	@echo "$'\033[7m==================================\033[0m"
	$(Q)make -C $(PLATFORMS_DIR)/$(ORCA_PLATFORM) -j 8
	$(Q)cp $(PLATFORMS_DIR)/$(ORCA_PLATFORM)/bin/$(PLATFORM_BIN) $(BINARY_DIR)/$(PLATFORM_BIN)

#Make documentation by invoking doxygen using the provided doxyfile.
#Generated documentation will be deployed to /docs folder.
documentation:
	@echo "$'\033[7m==================================\033[0m"
	@echo "$'\033[7m    Building API Documentation    \033[0m"
	@echo "$'\033[7m==================================\033[0m"
	$(Q)doxygen

#Generate scripts for log visualization. For each processing core, a two log files are writen
#to the /logs folder, one for UART output, and another one for Debugging output. Generated 
#scripts will try to invoke multitail on these file, so make sure that multitail is installed.
vismtail:
	@echo "#!/bin/sh" > $(BINARY_DIR)/output-debug.sh
	@echo "multitail ./logs/*debug.log -s $(ORCA_NOC_WIDTH)" \
		>> $(BINARY_DIR)/output-debug.sh
	@echo "#!/bin/sh" > $(BINARY_DIR)/output-uart.sh
	@echo "multitail ./logs/*uart.log -s $(ORCA_NOC_WIDTH)" \
		>> $(BINARY_DIR)/output-uart.sh

clean:
	@echo "$'\033[7m==================================\033[0m"
	@echo "$'\033[7m          Cleaning up...          \033[0m"
	@echo "$'\033[7m==================================\033[0m"
	$(Q)make -C $(URSA_DIR) clean
	$(Q)make -C $(MODELS_DIR) clean
	$(Q)make -C $(GDBRSP_DIR) clean
	$(Q)make -C $(PLATFORMS_DIR)/$(ORCA_PLATFORM) clean
	$(Q)rm -rf $(BINARY_DIR)/*.exe $(BINARY_DIR)/*.a $(BINARY_DIR)/*.o \
		$(BINARY_DIR)/*~ $(BINARY_DIR)/*.elf $(BINARY_DIR)/*.bin \
		$(BINARY_DIR)/*.cnt $(BINARY_DIR)/*.lst $(BINARY_DIR)/*.sec \
		$(BINARY_DIR)/*.txt $(BINARY_DIR)/*.sh
	$(Q)rm -rf docs/doxygen/
	$(Q)rm -rf logs/*.log
