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
ORCA_GLOBAL_FLAGS := -Wall -Wextra -Werror -g -std=c++17 \
	-O3 -march=native -mtune=native 

# Select the target platform. Different hardware architectures
# are deployed to /platforms folder. We currently provide two
# example architectures: (i) a single-core riscv processor core
# based on the HF-RiscV processor and (ii) a NoC-based 
# mesh-topologic manycore that uses the same processor core. 
# PLATFORM := (orca-mpsoc | single-core)
ORCA_PLATFORM := orca-mpsoc

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
# NETWORK ON-CHIP OPTIONS                                        #
#================================================================#

# Height (maximum x-axis coordinate -1) of the network-on-chip.
# Cannot be zero. Please note that the total number of tiles 
# (ORCA_NOC_HEIGHT * ORCA_NOC_WIDTH) cannot exceed MAX_UINT16.
ORCA_NOC_HEIGHT := 3

# Height (maximum y-axis coordinate -1) of the network-on-chip.
# Cannot be zero. Please note that the total number of tiles 
# (ORCA_NOC_HEIGHT * ORCA_NOC_WIDTH) cannot exceed MAX_UINT16.
ORCA_NOC_WIDTH  := 2

# Configures the quantity of flits that buffers can store before
# reaching overflow. Memory usage increases for large buffer, 
# although not impacting in simulation performance.
ORCA_BUFFER_CAPACITY := 8

# Set it to YES to enforce the simulation to check for buffers'
# sizes before pushing data (depletes simulation performance).
ORCA_CHECK_BUFFER_OVERFLOW := NO

# Set it to YES to enforce the simulation to check for buffers'
# sizes when pulling data. (depletes simulation performance). 
ORCA_CHECK_BUFFER_UNDERFLOW := NO

#================================================================#
# OFF-CHIP COMMUNICATION OPTIONS                                 #
#================================================================#

# Check whether destination port is connected when tranfering
# flits. Transfering flit to routers not mapped into the topology
# results will crash the simulator if this option if set to NO. 
# Setting it YES depletes performance. 
ORCA_CHECK_ROUTER_PORTS := NO

# These options control the log for packets passing through the 
# off-chip communication interface. Set this option to NO to
# disable logging, MINIMAL to print short messages, or FULL to 
# display the content of every packet. Please note that this may
# slow down the simulation.
ORCA_OFFCHIP_LOG_IN  := NO
ORCA_OFFCHIP_LOG_OUT := NO

# Sets the IP address for the server that handles packets coming
# from outside the chip. All tiles handling packets will share 
# the same server address. Setting this to \"127.0.0.1\" must 
# work for most systems. 
ORCA_OFFCHIP_SERVER_IP := \"127.0.0.1\"

# Sets the base port number for servers handling packets from
# outside the chip. The port to be used for each server is an
# increment on the base port. For example, core #2 will serve
# port 5002 if base port is set to 5000.
ORCA_OFFCHIP_SERVER_PORT  := 8888

# Sets the IP address to where the packets going to outside the
# chip must be delivered. Set it to \"127.0.0.1\" if running
# the client application in the same host as the simulation
ORCA_OFFCHIP_CLIENT_IP := \"127.0.0.1\"
ORCA_OFFCHIP_CLIENT_PORT := 9999

#================================================================#
# MEMORY CORE OPTIONS                                            #
#================================================================#

# Lowest readable/writable memory address. This option applies to
# all memory cores in the platform.
ORCA_MEMORY_BASE := 0x40000000

# Memory size (in bytes). This option applies to all memory cores
# in the platform. Syntax must be one of the following:
# 1000000     -- nearly 1MB sized 
# 0xFFFFF     -- exactly 1MB sized
# pow(10,6)   -- same as 1000000 (~1MB)
ORCA_MEMORY_SIZE := 1000000

# Set this option to YES to force the simulator to check the li-
# mits of memory space when reading or writing. This option is 
# useful for debugging memory access, but dramatically depletes
# simulation performance. 
ORCA_CHECK_MEMORY_SPACE := NO

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
# HARDWARE COUNTERS                                              #
#================================================================#

# Enable hardware counter for several hardware components. These 
# counter can be used to estimate the energy consumption of the 
# platform. Enabling counters depletes performance.

# Counts the number cycle in which the router tranfers flits.
ORCA_HWCOUNTERS_ROUTERS := NO

# Counts the number of readings and writings.
ORCA_HWCOUNTERS_MEMORY := NO

# Counts the number of instructions per instruction class.
ORCA_HWCOUNTERS_HFRISCV := NO

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

COMPLINE := $(COMPLINE) \
	-DORCA_NOC_HEIGHT=$(ORCA_NOC_HEIGHT) \
	-DORCA_NOC_WIDTH=$(ORCA_NOC_WIDTH) \
	-DORCA_EPOCH_LENGTH=$(ORCA_EPOCH_LENGTH)

#URSA parameters (code uses URSA instead of ORCA)
ifeq ($(ORCA_ZERO_TIME_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_ZERO_TIME_CHECKING
endif
ifeq ($(ORCA_QUEUE_SIZE_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_QUEUE_SIZE_CHECKING
endif

#buffer parameters
ifeq ($(ORCA_BUFFER_OVERFLOW_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DBUFFER_OVERFLOW_CHECKING
endif
ifeq ($(ORCA_BUFFER_UNDERFLOW_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DBUFFER_UNDERFLOW_CHECKING
endif

COMPLINE := $(COMPLINE) -DBUFFER_CAPACITY=$(ORCA_BUFFER_CAPACITY)

#router
ifeq ($(ORCA_GROUNDED_PORTS_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DROUTER_PORT_CONNECTED_CHECKING
endif

#off-chip
ifeq ($(ORCA_OFFCHIP_LOG_IN), YES)
	COMPLINE := $(COMPLINE) -DNETBRIDGE_ENABLE_LOG_INPUT
endif
ifeq ($(ORCA_OFFCHIP_LOG_OUT) ), YES)
	COMPLINE := $(COMPLINE) -DNETBRIDGE_ENABLE_LOG_OUTPUT
endif

COMPLINE := $(COMPLINE) \
	-DNETSOCKET_CLIENT_ADDRESS=$(ORCA_OFFCHIP_CLIENT_IP) \
	-DNETSOCKET_CLIENT_PORT=$(ORCA_OFFCHIP_CLIENT_PORT) \
	-DNETSOCKET_SERVER_ADDRESS=$(ORCA_OFFCHIP_SERVER_IP) \
	-DNETSOCKET_SERVER_PORT=$(ORCA_OFFCHIP_SERVER_PORT)

#memory
COMPLINE := $(COMPLINE) \
	-DORCA_MEMORY_BASE=$(ORCA_MEMORY_BASE) \
	-DORCA_MEMORY_SIZE=$(ORCA_MEMORY_SIZE)

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

#counters
ifeq ($(ORCA_HWCOUNTERS_MEMORY), YES)
	COMPLINE := $(COMPLINE) -DMEMORY_ENABLE_COUNTERS
endif

ifeq ($(ORCA_HWCOUNTERS_HFRISCV), YES)
	COMPLINE := $(COMPLINE) -DHFRISCV_ENABLE_COUNTERS
endif

#router parameters
ifeq ($(ORCA_HWCOUNTERS_ROUTERS), YES)
	COMPLINE := $(COMPLINE) -DROUTER_ENABLE_COUNTERS
endif

#gdbrsp
ifeq ($(ORCA_ENABLE_GDBRSP), YES)
	COMPLINE := $(COMPLINE) -DORCA_ENABLE_GDBRSP \
		 -DORCA_GDBRSP_PORT=$(ORCA_GDBRSP_PORT)
endif

export COMPLINE
export ORCA_NOC_HEIGHT
export ORCA_NOC_WIDTH
