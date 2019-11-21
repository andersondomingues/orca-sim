#configuration
#PLATFORM         := orca-generic
PLATFORM          := orca-dma
APPLICATIONS_DIR := applications

#libnames
PLATFORM_BIN      := $(PLATFORM).exe
SIMULATOR_LIB     := libsim.a
MODELS_LIB        := libmod.a
IMAGE_BIN         := image.bin

#include optmizations
include ./Configuration.mk

#directory configurations (paths)
SIMULATOR_DIR := $(CURDIR)/simulator
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
all: $(BINARY_DIR)/$(PLATFORM_BIN) $(BINARY_DIR)/$(IMAGE_BIN) vismtail
	@echo "$'\e[7m====================================\e[0m"
	@echo "$'\e[7m  All done! Starting simulation...  \e[0m"
	@echo "$'\e[7m====================================\e[0m"
	@echo " => lauching $(PLATFORM) instance:"
	$(BINARY_DIR)/$(PLATFORM_BIN) $(BINARY_DIR)/$(IMAGE_BIN) 

#URSA's simulation library
$(BINARY_DIR)/$(SIMULATOR_LIB):
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m     Building URSA's libsim       \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	make -C $(SIMULATOR_DIR) -j 8
	cp $(SIMULATOR_DIR)/bin/$(SIMULATOR_LIB) $(BINARY_DIR)/$(SIMULATOR_LIB)

#library containing hardware models
$(BINARY_DIR)/$(MODELS_LIB): $(BINARY_DIR)/$(SIMULATOR_LIB)
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m     Building hardware models     \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	make -C $(MODELS_DIR) -j 8
	cp $(MODELS_DIR)/bin/$(MODELS_LIB) $(BINARY_DIR)/$(MODELS_LIB)

#platform executable
$(BINARY_DIR)/$(PLATFORM_BIN): $(BINARY_DIR)/$(SIMULATOR_LIB) $(BINARY_DIR)/$(MODELS_LIB)
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m     Building the platform        \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	make -C $(PLATFORMS_DIR)/$(PLATFORM) -j 8
	cp $(PLATFORMS_DIR)/$(PLATFORM)/bin/$(PLATFORM_BIN) $(BINARY_DIR)/$(PLATFORM_BIN)

#software (kernel + loader)
$(BINARY_DIR)/$(IMAGE_BIN):
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m Building software (kernel + apps)\e[0m"
	@echo "$'\e[7m==================================\e[0m"	
	make -C $(SOFTWARE_DIR) $(IMAGE_BIN)

#documentation
#last line refers to a bug in tabu.sty. A replacement for
#the file is provided within /tools folder and should be 
#automatically applied while building
documentation:
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m    Building API Documentation    \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	doxygen
	cp ./tools/tabu.sty ./docs/doxygen/latex/ -rf

#visualization file for multitail
vismtail:
	@echo "#!/bin/sh" > ./tools/output-debug.sh
	@echo "multitail ./logs/*debug.log -s $(ORCA_NOC_WIDTH)" \
		>> ./tools/output-debug.sh
	@echo "#!/bin/sh" > ./tools/output-uart.sh
	@echo "multitail ./logs/*uart.log -s $(ORCA_NOC_WIDTH)" \
		>> ./tools/output-uart.sh

#generate hex files from memdumps
bp:
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m Generating Dumps, Please Wait... \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	@cd breakpoints; bash ../tools/bin-to-hex.sh; rm -rf *.bin
	@echo "Done."

#compile all tools at once
tools:
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m   Building tools, Please Wait... \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	make -C tools/orca-udptest

clean:
	@echo "$'\e[7m==================================\e[0m"
	@echo "$'\e[7m          Cleaning up...          \e[0m"
	@echo "$'\e[7m==================================\e[0m"
	@echo $(COMPLINE)
	@echo "$'\e[7m==================================\e[0m"
	@make -C $(SIMULATOR_DIR) clean
	@make -C $(MODELS_DIR) clean
	@make -C $(PLATFORMS_DIR)/$(PLATFORM) clean
	@make -C $(SOFTWARE_DIR) clean
	@rm -rf $(BINARY_DIR)/*.exe $(BINARY_DIR)/*.a $(BINARY_DIR)/*.o \
		$(BINARY_DIR)/*~ $(BINARY_DIR)/*.elf $(BINARY_DIR)/*.bin \
		$(BINARY_DIR)/*.cnt $(BINARY_DIR)/*.lst $(BINARY_DIR)/*.sec $(BINARY_DIR)/*.txt
	@rm -rf docs/doxygen/
	@rm -rf breakpoints/*.bin breakpoints/*.hex
	@make -C tools/orca-udptest clean
