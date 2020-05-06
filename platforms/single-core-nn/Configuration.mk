#================================================================#
# HARDWARE COUNTERS                                              #
#================================================================#

# Enable hardware counter for several hardware components. These 
# counter can be used to estimate the energy consumption of the 
# platform. Enabling counters depletes performance.

# Counts the number of readings and writings.
ORCA_HWCOUNTERS_MEMORY := NO

# Counts the number of instructions per instruction class.
ORCA_HWCOUNTERS_HFRISCV := YES

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


# ========================================================================
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.
# DO NOT MODIFY BELOW THIS LINE!
# ========================================================================
#counters
ifeq ($(ORCA_HWCOUNTERS_MEMORY), YES)
	COMPLINE := $(COMPLINE) -DMEMORY_ENABLE_COUNTERS
endif
ifeq ($(ORCA_HWCOUNTERS_HFRISCV), YES)
	COMPLINE := $(COMPLINE) -DHFRISCV_ENABLE_COUNTERS
endif

#memory
COMPLINE := $(COMPLINE) \
	-DORCA_MEMORY_BASE=$(ORCA_MEMORY_BASE) \
	-DORCA_MEMORY_SIZE=$(ORCA_MEMORY_SIZE)

export COMPLINE
