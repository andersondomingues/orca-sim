#================================================================#
# COMPILING OPTIONS                                              #
#================================================================#

# Below options will be set for all compilation within the
# project. 
# -Wall, -Wextra, -Werror: activates extra reporting 
# -g: adds debugging information
# -std=c++17: set C++ standard (required for old compilers)
# -O3, -march=native, -mtune=native: optmizations
# -lasan, -fsanitize=address: additional debugging support
ORCA_GLOBAL_FLAGS := -Wall -Wextra -Werror -g3 -std=c++17 \
	-O3 -march=native -mtune=native 

# Select the target platform. Different hardware architectures
# are deployed to /platforms folder. 
# allowed values:
# "orca-mpsoc", a NORMA-based manycore
# "single-core", a single hfriscv processor core
# "single-core-nn", same as above but with an nn accelerator
# "hfriscv-with-extcomm", single-core with external comm. module
ORCA_PLATFORM := hfriscv-with-extcomm
#ORCA_PLATFORM := single-core

#================================================================#
# SIMULATION OPTIONS                                             #
#================================================================#

# Number of cycles to be simulated before URSA's engine flush 
# the simulation queue. Larger number may present more accurate
# speed and a more accurate speed reporting, although consuming 
# more memory. Must be between (TILES * 4) and MAX_INT.
ORCA_EPOCH_LENGTH := 10000000

# Number of epochs to simulate. If set INF, simulation will run 
# until it receives a CTRL+C (sigint) interruption. Please note
# that one interruption is enough to stop the simulation, which 
# may last until the end of the current epoch. Another interrupt-
# ion will kill the application, skipping the reporting step
# (sigkill). When set to a positive value, the simulation engine
# execute for ORCA_EPOCH_LENGTH times ORCA_EPOCHS_TO_SIM cycles.
ORCA_EPOCHS_TO_SIM := INF

#================================================================#
# GDBRSP SUPPORT                                                 #
#================================================================#

# Enables support for GDB remote debugging processor cores. When
# enabled, GDBRSP module will pause cpu execution at their first
# cycle. To resume the cpu(s), connect to the corresponding. Ena-
# bling GDB support dramatically reduce simulation speed.
ORCA_ENABLE_GDBRSP := NO

# When enabled, the GDBRSP service will instantiate a UDP server
# for each processing tile. Each server will serve one UDP port,
# with core #0 starting with ORCA_GDBRSP_PORT. All other cores 
# will be increments on the base port (e.g., core #5 will serve at
# port = (ORCA_GDBRSP_PORT + 5).
ORCA_GDBRSP_PORT := 5000

#================================================================#
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.              #
# DO NOT MODIFY BELOW THIS LINE!                                 # 
#================================================================#

#ORCA_GLOBAL_FLAGS
MAIN_COMPLINE := $(ORCA_GLOBAL_FLAGS)

#SIMULATION FLAGS
ifneq ($(ORCA_EPOCHS_TO_SIM), INF)
	MAIN_COMPLINE := $(MAIN_COMPLINE) -DORCA_EPOCHS_TO_SIM=$(ORCA_EPOCHS_TO_SIM)
endif

MAIN_COMPLINE := $(MAIN_COMPLINE) -DORCA_EPOCH_LENGTH=$(ORCA_EPOCH_LENGTH)

#GDBRSP
ifeq ($(ORCA_ENABLE_GDBRSP), YES)
	MAIN_COMPLINE := $(MAIN_COMPLINE) -DORCA_ENABLE_GDBRSP \
		 -DORCA_GDBRSP_PORT=$(ORCA_GDBRSP_PORT)
endif

export MAIN_COMPLINE
