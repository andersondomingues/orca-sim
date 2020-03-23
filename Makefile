# Includes parameters from the configuration file
include ./Configuration.mk

# Name of URSA library, compiled as a static library.
URSA_LIB      := libsim.a

# Name of the library containing the hardware models. 
MODELS_LIB    := libmod.a

# Additional parameters (do not modify them unless you know
# what you are doing here).
PLATFORM_BIN      := $(PLATFORM).exe

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
export Q := @
# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory
endif

#include optmizations
include ./Configuration.mk

#directory configurations (paths)
URSA_DIR      := $(CURDIR)/simulator
BINARY_DIR    := $(CURDIR)/bin
PLATFORMS_DIR := $(CURDIR)/platforms
MODELS_DIR    := $(CURDIR)/models
TOOLS_DIR     := $(CURDIR)/tools
SOFTWARE_DIR  := $(CURDIR)/software

#phonies (see https://www.gnu.org/software/make/manual/html_node/Phony-Targets.html)
.PHONY: clean documentation multitail tools

#compile everything if necessary and run
#simulatotion requires the simulator and software 
# - software has no dependencies
# - simulator has no dependencies
# - hardware models depends on the simulator
# - platform depends on simulator and hardware models
# - visualization file for multitail has no dependency
all: $(BINARY_DIR)/$(PLATFORM_BIN) vismtail
	@echo "$'\e[7m====================================\e[0m"
	@echo "$'\e[7m  All done!                         \e[0m"
	@echo "$'\e[7m====================================\e[0m"
	@echo " => simulation tool $(PLATFORM_BIN) is ready."

#URSA's simulation library
$(BINARY_DIR)/$(URSA_LIB): $(URSA_DIR)/src/*.cpp  $(URSA_DIR)/include/*.h 
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m     Building URSA's libsim       \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	$(Q)make -C $(URSA_DIR) -j 8
	$(Q)cp $(URSA_DIR)/bin/$(URSA_LIB) $(BINARY_DIR)/$(URSA_LIB)

#library containing hardware models
$(BINARY_DIR)/$(MODELS_LIB): $(BINARY_DIR)/$(URSA_LIB) $(MODELS_DIR)/src/*.cpp  $(MODELS_DIR)/include/*.h
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m     Building hardware models     \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	$(Q)make -C $(MODELS_DIR) -j 8
	$(Q)cp $(MODELS_DIR)/bin/$(MODELS_LIB) $(BINARY_DIR)/$(MODELS_LIB)

#platform executable
$(BINARY_DIR)/$(PLATFORM_BIN): $(BINARY_DIR)/$(URSA_LIB) $(BINARY_DIR)/$(MODELS_LIB) $(PLATFORMS_DIR)/$(PLATFORM)/src/*.cpp  $(PLATFORMS_DIR)/$(PLATFORM)/include/*.h
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m     Building the platform        \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	$(Q)make -C $(PLATFORMS_DIR)/$(PLATFORM) -j 8
	$(Q)cp $(PLATFORMS_DIR)/$(PLATFORM)/bin/$(PLATFORM_BIN) $(BINARY_DIR)/$(PLATFORM_BIN)

#documentation
#last line refers to a bug in tabu.sty. A replacement for
#the file is provided within /tools folder and should be 
#automatically applied while building
documentation:
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m    Building API Documentation    \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	$(Q)doxygen
	$(Q)cp ./tools/tabu.sty ./docs/doxygen/latex/ -rf

#generete script for multitail (visualization, requires multitail)
vismtail:
	@echo "#!/bin/sh" > $(BINARY_DIR)/output-debug.sh
	@echo "multitail ./logs/*debug.log -s $(ORCA_NOC_WIDTH)" \
		>> $(BINARY_DIR)/output-debug.sh
	@echo "#!/bin/sh" > $(BINARY_DIR)/output-uart.sh
	@echo "multitail ./logs/*uart.log -s $(ORCA_NOC_WIDTH)" \
		>> $(BINARY_DIR)/output-uart.sh

#generate hex files from memdumps
bp:
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m Generating Dumps, Please Wait... \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	@cd breakpoints; bash ../tools/bin-to-hex.sh; rm -rf *.bin
	@echo "Done."

clean:
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m          Cleaning up...          \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	$(Q)make -C $(URSA_DIR) clean
	$(Q)make -C $(MODELS_DIR) clean
	$(Q)make -C $(PLATFORMS_DIR)/$(PLATFORM) clean
	$(Q)rm -rf $(BINARY_DIR)/*.exe $(BINARY_DIR)/*.a $(BINARY_DIR)/*.o \
		$(BINARY_DIR)/*~ $(BINARY_DIR)/*.elf $(BINARY_DIR)/*.bin \
		$(BINARY_DIR)/*.cnt $(BINARY_DIR)/*.lst $(BINARY_DIR)/*.sec \
		$(BINARY_DIR)/*.txt $(BINARY_DIR)/*.sh
	$(Q)rm -rf docs/doxygen/
	$(Q)rm -rf logs/*.log
	$(Q)rm -rf breakpoints/*.bin breakpoints/*.hex
