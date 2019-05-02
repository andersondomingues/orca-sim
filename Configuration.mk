# =======================================================[ GLOBAL SETTINGS ]
# Flags set here will be propagated to all modules, including
GLOBAL_SETTINGS := -O3 -Wall -Wextra -Werror -g -std=c++11

# ==================================================================[ ORCA ]
# Width (x-axis coordinate) of the network-on-chip. Cannot be zero,
# otherwise simulation won't compile.
ORCA_NOC_HEIGHT := 6

# Width (y-axis coordinate) of the network-on-chip. Cannot be zero,
# otherwise simulation won't compile.
ORCA_NOC_WIDTH  := 6

# Number of cycles before calling the frequency analisys tool. Shorter
# values may compromise the performance of the simulation, while higher
# values may provide inaccurate measurements of the achieved frequency.
ORCA_PULSE_LENGTH  := 1000000

# Number of pulses to simulate. Set to INF to simulate indefinitely.
ORCA_PULSES_TO_SIM := 10

# ===========================================================[ URSA ENGINE ]
# Check whether some event has been schedule to run in some point of 
# time prior to the beggining. First event must always be schedule to 
# time=1, otherwise the simulation will crash (depletes performance).
URSA_ZERO_TIME_CHECKING := NO

# Check whether the queue has some event before simulating the next event.
# When simulating hardware, the number of events in the queue is constant,
# so checking the size of the queue is unecessary. Set this option to 
# YES to force checking (depletes performance).
URSA_QUEUE_SIZE_CHECKING := NO

# =============================================================[ NETSOCKET ]
# Outputs log for outgoing packets (depletes performance).
NETSOCKET_LOG_OUTGOING_PACKETS := NO

# Outputs log for incoming packets (depletes performance).
NETSOCKET_LOG_INCOMING_PACKETS := NO

# Sets client UDP/IP address and port for the netsocket. This is the address
# that the netsocket will connect when sending packets to outside the noc.
NETSOCKET_CLIENT_ADDRESS := \"127.0.0.1\"
NETSOCKET_CLIENT_PORT := 8888

# Sets server IP address and port for the netsocket. This is the address
# that application would use to send packets to the internal network.
NETSOCKET_SERVER_ADDRESS := \"127.0.0.1\"
NETSOCKET_SERVER_PORT := 9999

# ===============================================================[ BUFFER ]
# Check whether the buffer is full before pushing data (depletes performance).
BUFFER_OVERFLOW_CHECKING := NO

# Check whether the buffer is empty before popping data (depletes performance).
BUFFER_UNDERFLOW_CHECKINGmemory := NO

# ===============================================================[ MEMORY ]
# Check whether address are mapped to some memory range before writing
# to memory. Set to YES to force checking (depletes performance).
MEMORY_WRITE_ADDRESS_CHECKING := NO

# Check whether address are mapped to some memory range before reading from
# memory. Set to YES to force checking (depletes performance).
MEMORY_READ_ADDRESS_CHECKING := NO

# Check whether address are mapped to some memory range before wipeing
# memory ranges. Set to YES to force checking (depletes performance).
MEMORY_WIPE_ADDRESS_CHECKING := NO

# Enable counter for read and write operations (depletes performance).
MEMORY_ENABLE_COUNTERS := YES

# ==============================================================[ HFRISCV ]
# Check whether address are mapped to some memory range before writing
# to memory. Set to YES to force checking (depletes performance). This
# option does not override the one in memory module.
HFRISCV_WRITE_ADDRESS_CHECKING := NO

# Check whether address are mapped to some memory range before reading
# from memory. Set to YES to force checking (depletes performance). This
# option does not override the one in memory module.
HFRISCV_READ_ADDRESS_CHECKING := NO

# Enable counter for instructions' classes (depletes performance).
HFRISCV_ENABLE_COUNTERS := YES

# ==============================================================[ ROUTER ]
# Enable counters for number of active cycles
ROUTER_ENABLE_COUNTERS := YES

# Check whether destination port is connected when tranfering flits.
# Transfering flit to routers not mapped into the topology results in
# crash. Set to YES to force checking (depletes performance).
ROUTER_PORT_CONNECTED_CHECKING := NO

# ========================================================================
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.
# DO NOT MODIFY BELOW THIS LINE!
# ========================================================================

#global parameters
COMPLINE := $(COMPLINE)$(GLOBAL_SETTINGS)

#ORCA parameters
ifneq ($(ORCA_PULSES_TO_SIM), INF)
COMPLINE := $(COMPLINE) -DORCA_PULSES_TO_SIM=$(ORCA_PULSES_TO_SIM)
endif

COMPLINE := $(COMPLINE) \
	-DORCA_NOC_HEIGHT=$(ORCA_NOC_HEIGHT) \
	-DORCA_NOC_WIDTH=$(ORCA_NOC_WIDTH) \
	-DORCA_PULSE_LENGTH=$(ORCA_PULSE_LENGTH)

#URSA parameters
ifeq ($(URSA_ZERO_TIME_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_ZERO_TIME_CHECKING
endif
ifeq ($(URSA_QUEUE_SIZE_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_QUEUE_SIZE_CHECKING
endif

#netsocket parameters
ifeq ($(NETSOCKET_LOG_OUTGOING_PACKETS), YES)
	COMPLINE := $(COMPLINE) -DNETSOCKET_LOG_OUTGOING_PACKETS
endif
ifeq ($(NETSOCKET_LOG_INCOMING_PACKETS), YES)
	COMPLINE := $(COMPLINE) -DNETSOCKET_LOG_INCOMING_PACKETS
endif

COMPLINE := $(COMPLINE) \
	-DNETSOCKET_CLIENT_ADDRESS=$(NETSOCKET_CLIENT_ADDRESS) \
	-DNETSOCKET_CLIENT_PORT=$(NETSOCKET_CLIENT_PORT) \
	-DNETSOCKET_SERVER_ADDRESS=$(NETSOCKET_SERVER_ADDRESS) \
	-DNETSOCKET_SERVER_PORT=$(NETSOCKET_SERVER_PORT)

#buffer parameters
ifeq ($(BUFFER_OVERFLOW_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DBUFFER_OVERFLOW_CHECKING
endif
ifeq ($(BUFFER_UNDERFLOW_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DBUFFER_UNDERFLOW_CHECKING
endif

#memory parameters
ifeq ($(MEMORY_WRITE_ADDRESS_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DMEMORY_WRITE_ADDRESS_CHECKING
endif
ifeq ($(MEMORY_READ_ADDRESS_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DMEMORY_READ_ADDRESS_CHECKING
endif
ifeq ($(MEMORY_WIPE_ADDRESS_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DMEMORY_WIPE_ADDRESS_CHECKING
endif
ifeq ($(MEMORY_ENABLE_COUNTERS), YES)
	COMPLINE := $(COMPLINE) -DMEMORY_ENABLE_COUNTERS
endif

#hfriscv parameters
ifeq ($(HFRISCV_WRITE_ADDRESS_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DHFRISCV_WRITE_ADDRESS_CHECKING
endif
ifeq ($(HFRISCV_READ_ADDRESS_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DHFRISCV_READ_ADDRESS_CHECKING
endif
ifeq ($(HFRISCV_ENABLE_COUNTERS), YES)
	COMPLINE := $(COMPLINE) -DHFRISCV_ENABLE_COUNTERS
endif

#router parameters
ifeq ($(ROUTER_ENABLE_COUNTERS), YES)
	COMPLINE := $(COMPLINE) -DROUTER_ENABLE_COUNTERS
endif
ifeq ($(ROUTER_PORT_CONNECTED_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DROUTER_PORT_CONNECTED_CHECKING
endif

export COMPLINE
