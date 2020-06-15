#================================================================#
# BUFFER                                                         #
#================================================================#

# Set it to YES to enforce the simulation to check for buffers'
# sizes before pushing data (depletes simulation performance).
ORCA_CHECK_BUFFER_OVERFLOW := NO

# Set it to YES to enforce the simulation to check for buffers'
# sizes when pulling data. (depletes simulation performance). 
ORCA_CHECK_BUFFER_UNDERFLOW := NO

# Configures the quantity of flits that buffers can store before
# reaching overflow. Memory usage increases for large buffer,
# although not impacting in simulation performance.
ORCA_BUFFER_CAPACITY := 8

#================================================================#
# ROUTERS                                                        #
#================================================================#

# Check whether destination port is connected when tranfering
# flits. Transfering flit to routers not mapped into the topology
# results will crash the simulator if this option if set to NO. 
# Setting it YES depletes performance. 
ORCA_CHECK_ROUTER_PORTS := NO

#================================================================#
# MEMORY CORE                                                    #
#================================================================#

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

#================================================================#
# HARDWARE COUNTERS                                              #
#================================================================#

# Enable hardware counter for several hardware components. These 
# counter can be used to estimate the energy consumption of the 
# platform. Enabling counters depletes performance.

# Counts the number of readings and writings.
ORCA_HWCOUNTERS_MEMORY := YES

# Counts the number of readings and writings for the 
# two auxiliary memory module included in the network
# interface. Requires ORCA_HW_COUNTERS_MEMORY to be 
# enabled.
ORCA_HWCOUNTERS_NI := NO

# Counts the number of instructions per instruction class.
ORCA_HWCOUNTERS_HFRISCV := YES

# Enable hardware counter for several hardware components. These 
# counter can be used to estimate the energy consumption of the 
# platform. Enabling counters depletes performance.
# Counts the number cycle in which the router tranfers flits.
ORCA_HWCOUNTERS_ROUTERS := NO

#================================================================#
# OFF-CHIP COMMUNICATION OPTIONS                                 #
#================================================================#

# These options control the log for packets passing through the
# off-chip communication interface. Set this option to NO to
# disable logging, MINIMAL to print short messages, or FULL to
# display the content of every packet. Please note that this may
# slow down the simulation.
ORCA_OFFCHIP_LOG_IN  := YES
ORCA_OFFCHIP_LOG_OUT := YES

# Sets the IP address for the server that handles packets coming
# from outside the chip. All tiles handling packets will share
# the same server address. Setting this to \"127.0.0.1\" must
# work for most systems.
ORCA_OFFCHIP_SERVER_IP :=\"127.0.0.1\"

# Sets the base port number for servers handling packets from
# outside the chip. The port to be used for each server is an
# increment on the base port. For example, core #2 will serve
# port 5002 if base port is set to 5000.
ORCA_OFFCHIP_SERVER_PORT := 8888

# Sets the IP address to where the packets going to outside the
# chip must be delivered. Set it to \"127.0.0.1\" if running
# the client application in the same host as the simulation
ORCA_OFFCHIP_CLIENT_IP :=\"127.0.0.1\"
ORCA_OFFCHIP_CLIENT_PORT := 9999

#================================================================#
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.              #
# DO NOT MODIFY BELOW THIS LINE!                                 #
#================================================================#

#OFF-CHIP
MODELS_COMPLINE := $(MODELS_COMPLINE) \
        -DNETSOCKET_CLIENT_ADDRESS=$(ORCA_OFFCHIP_CLIENT_IP) \
        -DNETSOCKET_CLIENT_PORT=$(ORCA_OFFCHIP_CLIENT_PORT) \
        -DNETSOCKET_SERVER_ADDRESS=$(ORCA_OFFCHIP_SERVER_IP) \
        -DNETSOCKET_SERVER_PORT=$(ORCA_OFFCHIP_SERVER_PORT)

ifeq ($(ORCA_OFFCHIP_LOG_IN), YES)
        MODELS_COMPLINE := $(MODELS_COMPLINE) -DNETBRIDGE_ENABLE_LOG_INPUT
endif
ifeq ($(ORCA_OFFCHIP_LOG_OUT) ), YES)
        MODELS_COMPLINE := $(MODELS_COMPLINE) -DNETBRIDGE_ENABLE_LOG_OUTPUT
endif

#BUFFERS
ifeq ($(ORCA_BUFFER_OVERFLOW_CHECKING), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DBUFFER_OVERFLOW_CHECKING
endif
ifeq ($(ORCA_BUFFER_UNDERFLOW_CHECKING), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DBUFFER_UNDERFLOW_CHECKING
endif

MODELS_COMPLINE := $(MODELS_COMPLINE) \
        -DBUFFER_CAPACITY=$(ORCA_BUFFER_CAPACITY)

#ROUTERS
ifeq ($(ORCA_GROUNDED_PORTS_CHECKING), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DROUTER_PORT_CONNECTED_CHECKING
endif

#MEMORY
ifeq ($(ORCA_MEMORY_SPACE_CHECKING), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) \
		-DMEMORY_WRITE_ADDRESS_CHECKING \
		-DMEMORY_READ_ADDRESS_CHECKING \
		-DMEMORY_WIPE_ADDRESS_CHECKING
endif

#RISCV
ifeq ($(ORCA_HFRISCV_MODE), CYCLE)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DHFRISCV_CYCLE_ACCURACY 
endif

#COUNTERS
ifeq ($(ORCA_HWCOUNTERS_MEMORY), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DMEMORY_ENABLE_COUNTERS
endif
ifeq ($(ORCA_HWCOUNTERS_HFRISCV), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DHFRISCV_ENABLE_COUNTERS
endif
ifeq ($(ORCA_HWCOUNTERS_ROUTERS), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DROUTER_ENABLE_COUNTERS
endif
ifeq ($(ORCA_HWCOUNTERS_NI), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DNI_ENABLE_COUNTERS
endif



export MODELS_COMPLINE
