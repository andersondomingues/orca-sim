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

# Apps to be compiled within kernel image. For multiple applications, 
# use spacebar to separate names. Applications defined here will not 
# be included in compilation unless you edit the file 
#          extensions/orca-core/src/orca-core.cpp,
# where you should set the spawn of tasks in each of the cores. 
ORCA_APPLICATIONS := producer-consumer-pubsub producer-consumer app-spawner app-bloater deadline-monitor

# Software extensions (experimental)
ORCA_EXTENSIONS := orca-core orca-pubsub orca-monitoring

# ============================================================[ HELLFIREOS ]
# Set level of logging for the HellfireOS kernel. 
# 0 => disabled 
# 1 => interruption and dispatch information (default)
# 2 => same as level one plus calls to kernel functions
KERNEL_LOG_LEVEL := 1

# ==================================================================[ ORCA ]
# Width (x-axis coordinate) of the network-on-chip. Cannot be zero,
# otherwise simulation won't compile.
ORCA_NOC_HEIGHT := 3
# Width (y-axis coordinate) of the network-on-chip. Cannot be zero,
# otherwise simulation won't compile.
ORCA_NOC_WIDTH  := 2

# Number of cycles before calling the frequency analisys tool. Shorter
# values may compromise the performance of the simulation, while higher
# values may provide inaccurate measurements of the achieved frequency.
ORCA_EPOCH_LENGTH  := 10000000
#ORCA_EPOCH_LENGTH  := 50000000

# Number of pulses to simulate. Set to INF to simulate indefinitely.
ORCA_EPOCHS_TO_SIM := INF
#ORCA_EPOCHS_TO_SIM := 3

# Enable multithread simulation (experimental)
ORCA_ENABLE_MULTITHREADING := NO

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
# Outputs log for outgoing packets (slightly depletes performance).
NETSOCKET_LOG_OUTGOING_PACKETS := NO

# Outputs log for incoming packets (slightly depletes performance).
NETSOCKET_LOG_INCOMING_PACKETS := NO

NETBRIDGE_ENABLE_LOG_INPUT  := YES
NETBRIDGE_ENABLE_LOG_OUTPUT := YES

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
BUFFER_UNDERFLOW_CHECKING := NO

# Configure the capacity of the buffers used within the system. To disable 
# network congestion, set this to a higher value. Please note that increasing t
# the capacity of buffers also increases the memory usage of the simulator. (
# The default for this option is 8 flits.
BUFFER_CAPACITY := 8

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
MEMORY_ENABLE_COUNTERS := NO

# ==============================================================[ HFRISCV ]
# Check whether address are mapped to some memory range before writing
# to memory. Set to YES to force checking (depletes performance). This
# option does not override the one in memory module.
# TODO: check whether this options works
HFRISCV_WRITE_ADDRESS_CHECKING := NO

# Check whether address are mapped to some memory range before reading
# from memory. Set to YES to force checking (depletes performance). This
# option does not override the one in memory module.
# TODO: check whether this options works
HFRISCV_READ_ADDRESS_CHECKING := NO

# Enable counter for instructions' classes (depletes performance).
HFRISCV_ENABLE_COUNTERS := YES

# Set the operation mode for the core, can be 
# either CYCLE or INSTRUCTION (default).
# - CYCLE: enable cycle counting per instruction and branch prediction
# - INSTRUCTION: all instruction take one cycle to exit the pipeline
#HFRISCV_MODE := CYCLE
HFRISCV_MODE := INSTRUCTION

# ==============================================================[ NETIF ]
# Check whether netif is writing to unmapped memory space
NETIF_WRITE_ADDRESS_CHECKING := NO

# check whether netif is reading from unmapped memory space
NETIF_READ_ADDRESS_CHECKING := NO

# ==============================================================[ ROUTER ]
# Enable counters for number of active cycles
ROUTER_ENABLE_COUNTERS := NO

# Check whether destination port is connected when tranfering flits.
# Transfering flit to routers not mapped into the topology results in
# crash. Set to YES to force checking (depletes performance).
ROUTER_PORT_CONNECTED_CHECKING := NO

# ========================================================================
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.
# DO NOT MODIFY BELOW THIS LINE!
# ========================================================================

#ORCA parameters
ifneq ($(ORCA_EPOCHS_TO_SIM), INF)
	COMPLINE := $(COMPLINE) -DORCA_EPOCHS_TO_SIM=$(ORCA_EPOCHS_TO_SIM)
endif

COMPLINE := $(COMPLINE) \
	-DORCA_NOC_HEIGHT=$(ORCA_NOC_HEIGHT) \
	-DORCA_NOC_WIDTH=$(ORCA_NOC_WIDTH) \
	-DORCA_EPOCH_LENGTH=$(ORCA_EPOCH_LENGTH)

ifeq ($(ORCA_ENABLE_MULTITHREADING), YES)
	COMPLINE := $(COMPLINE) -DORCA_ENABLE_MULTITHREADING
endif 

#URSA parameters
ifeq ($(URSA_ZERO_TIME_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_ZERO_TIME_CHECKING
endif
ifeq ($(URSA_QUEUE_SIZE_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DURSA_QUEUE_SIZE_CHECKING
endif

#netbridge parameters
ifeq ($(NETBRIDGE_ENABLE_LOG_INPUT), YES)
	COMPLINE := $(COMPLINE) -DNETBRIDGE_ENABLE_LOG_INPUT
endif
ifeq ($(NETBRIDGE_ENABLE_LOG_OUTPUT), YES)
	COMPLINE := $(COMPLINE) -DNETBRIDGE_ENABLE_LOG_OUTPUT
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

#netif parameters
ifeq ($(NETIF_WRITE_ADDRESS_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DNETIF_WRITE_ADDRESS_CHECKING
endif
ifeq ($(NETIF_READ_ADDRESS_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DNETIF_READ_ADDRESS_CHECKING
endif


#buffer parameters
ifeq ($(BUFFER_OVERFLOW_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DBUFFER_OVERFLOW_CHECKING
endif
ifeq ($(BUFFER_UNDERFLOW_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DBUFFER_UNDERFLOW_CHECKING
endif

COMPLINE := $(COMPLINE) \
	-DBUFFER_CAPACITY=$(BUFFER_CAPACITY)

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
ifeq ($(HFRISCV_MODE), CYCLE)
	COMPLINE := $(COMPLINE) -DHFRISCV_CYCLE_ACCURACY 
endif

#router parameters
ifeq ($(ROUTER_ENABLE_COUNTERS), YES)
	COMPLINE := $(COMPLINE) -DROUTER_ENABLE_COUNTERS
endif
ifeq ($(ROUTER_PORT_CONNECTED_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DROUTER_PORT_CONNECTED_CHECKING
endif

export COMPLINE
export GLOBAL_SETTINGS
export ORCA_NOC_HEIGHT
export ORCA_NOC_WIDTH
export ORCA_APPLICATIONS
export ORCA_EXTENSIONS
export KERNEL_LOG_LEVEL
