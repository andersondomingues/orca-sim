# =======================================================[ GLOBAL SETTINGS ]
# Flags we set here will be propagated to all hardware modules (not sw)
# Currently adopted flags:
# -Wall, -Wextra, Werror: activate a lot of warning at compile time
# -g: attach debug information to the generated executable
# -std: required by old GCC to set c++17 as default the c++ 
# -march, -mtune: optimize code for current machine architecture
# -lasan, -fsanitize: add memory sanitizer to code
GLOBAL_SETTINGS := -Wall -Wextra -Werror -g -std=c++17 -O3 -march=native -mtune=native 
#-lasan -fsanitize=address

# Select the target platform. Different hardware architectures
# are deployed to /platforms folder. We currently provide two
# example architectures: (i) a single-core riscv processor core
# based on the HF-RiscV processor and (ii) a NoC-based  
# mesh-topologic manycore that uses the same processor core . 
# PLATFORM := (orca-mpsoc | single-core | single-core-nn)
ORCA_PLATFORM  := single-core

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

# Set this option to YES to check event queue for invalid events.
# If not debugging scheduling, set this option to NO, as it deple-
# tes simulation performance.
ORCA_ZERO_TIME_CHECKING := NO

# Set this option to YES to check whether the queue is empty 
# during the simulation. If not debuggin scheduling, set this 
# option to NO, as it depletes simulation time. 
ORCA_QUEUE_SIZE_CHECKING := NO

#================================================================#
# MEMORY CORE OPTIONS                                            #
#================================================================#

# Set this option to YES to force the simulator to check the li-
# mits of memory space when reading or writing. This option is 
# useful for debugging memory access, but dramatically depletes
# simulation performance. 
ORCA_CHECK_MEMORY_SPACE := YES

#================================================================#
# HF-RISCV CORE OPTIONS                                          #
#================================================================#

# Set the operation mode for the HFRisc-V core. Two modes are 
# supported (INSTRUCTION is faster):
# - CYCLE: mimic pipeline by adding delay to instructions 
# - INSTRUCTION: all instruction take exacly one cycle to finish 
ORCA_HFRISCV_MODE := CYCLE

# Enables support for GDB remote debugging for processing tiles.
# Enabling GDB support dramatically reduce simulation speed.
ORCA_ENABLE_GDBRSP := NO

# When enabled, the GDBRSP service will instantiate a UDP server
# for each processing tile. Each server will serve one UDP port,
# with core #0 starting with ORCA_GDBRSP_PORT. All other cores 
# will be increments on the base port (e.g., core number five will
# serve at port = ORCA_GDBRSP_PORT + 5.
ORCA_GDBRSP_PORT := 5000


#================================================================#
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.              #
# DO NOT MODIFY BELOW THIS LINE!                                 # 
#================================================================#

#ORCA_GLOBAL_FLAGS
COMPLINE := $(ORCA_GLOBAL_FLAGS)

#simulation flags
ifneq ($(ORCA_EPOCHS_TO_SIM), INF)
	COMPLINE := $(COMPLINE) -DORCA_EPOCHS_TO_SIM=$(ORCA_EPOCHS_TO_SIM)
endif

COMPLINE := $(COMPLINE) -DORCA_EPOCH_LENGTH=$(ORCA_EPOCH_LENGTH)

#URSA parameters (code uses URSA instead of ORCA)
ifeq ($(ORCA_ZERO_TIME_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_ZERO_TIME_CHECKING
endif
ifeq ($(ORCA_QUEUE_SIZE_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_QUEUE_SIZE_CHECKING
endif

#memory
ifeq ($(ORCA_MEMORY_SPACE_CHECKING), YES)
	COMPLINE := $(COMPLINE) \
		-DMEMORY_WRITE_ADDRESS_CHECKING \
		-DMEMORY_READ_ADDRESS_CHECKING \
		-DMEMORY_WIPE_ADDRESS_CHECKING
endif

#riscv
ifeq ($(ORCA_HFRISCV_MODE), CYCLE)
	COMPLINE := $(COMPLINE) -DHFRISCV_CYCLE_ACCURACY 
endif

#gdbrsp
ifeq ($(ORCA_ENABLE_GDBRSP), YES)
	COMPLINE := $(COMPLINE) -DORCA_ENABLE_GDBRSP \
		 -DORCA_GDBRSP_PORT=$(ORCA_GDBRSP_PORT)
endif

export COMPLINE
