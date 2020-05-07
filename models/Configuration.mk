#================================================================#
# BUFFER SANITY CHECKING                                         #
#================================================================#

# Set it to YES to enforce the simulation to check for buffers'
# sizes before pushing data (depletes simulation performance).
ORCA_CHECK_BUFFER_OVERFLOW := NO

# Set it to YES to enforce the simulation to check for buffers'
# sizes when pulling data. (depletes simulation performance). 
ORCA_CHECK_BUFFER_UNDERFLOW := NO

#================================================================#
# ROUTERS SANITY CHECKING                                        #
#================================================================#

# Check whether destination port is connected when tranfering
# flits. Transfering flit to routers not mapped into the topology
# results will crash the simulator if this option if set to NO. 
# Setting it YES depletes performance. 
ORCA_CHECK_ROUTER_PORTS := NO

#================================================================#
# MEMORY CORE SANIRY CHECKING                                    #
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

#================================================================#
# HARDWARE COUNTERS                                              #
#================================================================#

# Enable hardware counter for several hardware components. These 
# counter can be used to estimate the energy consumption of the 
# platform. Enabling counters depletes performance.

# Counts the number of readings and writings.
ORCA_HWCOUNTERS_MEMORY := YES

# Counts the number of instructions per instruction class.
ORCA_HWCOUNTERS_HFRISCV := NO

# Enable hardware counter for several hardware components. These 
# counter can be used to estimate the energy consumption of the 
# platform. Enabling counters depletes performance.
# Counts the number cycle in which the router tranfers flits.
ORCA_HWCOUNTERS_ROUTERS := NO

#================================================================#
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.              #
# DO NOT MODIFY BELOW THIS LINE!                                 #
#================================================================#

#BUFFERS
ifeq ($(ORCA_BUFFER_OVERFLOW_CHECKING), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DBUFFER_OVERFLOW_CHECKING
endif
ifeq ($(ORCA_BUFFER_UNDERFLOW_CHECKING), YES)
	MODELS_COMPLINE := $(MODELS_COMPLINE) -DBUFFER_UNDERFLOW_CHECKING
endif

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

export MODELS_COMPLINE
