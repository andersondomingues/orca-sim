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
ORCA_MEMORY_SIZE := 0x008FFFFF

#================================================================#
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.              #
# DO NOT MODIFY BELOW THIS LINE!                                 #
#================================================================#

#MEMORY
PLAT_COMPLINE := \
	-DORCA_MEMORY_BASE=$(ORCA_MEMORY_BASE) \
	-DORCA_MEMORY_SIZE=$(ORCA_MEMORY_SIZE)

#NOC dimensions must be exported separately
export PLAT_COMPLINE
