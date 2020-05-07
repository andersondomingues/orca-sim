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

#================================================================#
# OFF-CHIP COMMUNICATION OPTIONS                                 #
#================================================================#

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

#================================================================#
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.              #
# DO NOT MODIFY BELOW THIS LINE!                                 #
#================================================================#

#NOC AND BUFFERS
PLAT_COMPLINE := -DORCA_NOC_HEIGHT=$(ORCA_NOC_HEIGHT) \
	-DORCA_NOC_WIDTH=$(ORCA_NOC_WIDTH) 

PLAT_COMPLINE := $(PLAT_COMPLINE) \
	-DBUFFER_CAPACITY=$(ORCA_BUFFER_CAPACITY)

#OFF-CHIP
ifeq ($(ORCA_OFFCHIP_LOG_IN), YES)
	PLAT_COMPLINE := $(PLAT_COMPLINE) -DNETBRIDGE_ENABLE_LOG_INPUT
endif
ifeq ($(ORCA_OFFCHIP_LOG_OUT) ), YES)
	PLAT_COMPLINE := $(PLAT_COMPLINE) -DNETBRIDGE_ENABLE_LOG_OUTPUT
endif

PLAT_COMPLINE := $(PLAT_COMPLINE) \
	-DNETSOCKET_CLIENT_ADDRESS=$(ORCA_OFFCHIP_CLIENT_IP) \
	-DNETSOCKET_CLIENT_PORT=$(ORCA_OFFCHIP_CLIENT_PORT) \
	-DNETSOCKET_SERVER_ADDRESS=$(ORCA_OFFCHIP_SERVER_IP) \
	-DNETSOCKET_SERVER_PORT=$(ORCA_OFFCHIP_SERVER_PORT)

#MEMORY
PLAT_COMPLINE := $(PLAT_COMPLINE) \
	-DORCA_MEMORY_BASE=$(ORCA_MEMORY_BASE) \
	-DORCA_MEMORY_SIZE=$(ORCA_MEMORY_SIZE)

#NOC dimensions must be exported separately
export PLAT_COMPLINE
export ORCA_NOC_HEIGHT
export ORCA_NOC_WIDTH
