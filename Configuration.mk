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

# ==================================================================[ ORCA ]

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


# ===============================================================[ MEMORY ]
# Check whether address are mapped to some memory range before writing
# to memory. Set to YES to force checking (depletes performance).
MEMORY_WRITE_ADDRESS_CHECKING := YES

# Check whether address are mapped to some memory range before reading from
# memory. Set to YES to force checking (depletes performance).
MEMORY_READ_ADDRESS_CHECKING := YES

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

# ========================================================================
# GENERATION OF COMPILATION PARAMETERS STARTS HERE.
# DO NOT MODIFY BELOW THIS LINE!
# ========================================================================

#ORCA parameters
ifneq ($(ORCA_EPOCHS_TO_SIM), INF)
	COMPLINE := $(COMPLINE) -DORCA_EPOCHS_TO_SIM=$(ORCA_EPOCHS_TO_SIM)
endif

COMPLINE := $(COMPLINE) \
	-DORCA_EPOCH_LENGTH=$(ORCA_EPOCH_LENGTH)
#	-DORCA_NOC_HEIGHT=$(ORCA_NOC_HEIGHT) \
#	-DORCA_NOC_WIDTH=$(ORCA_NOC_WIDTH) \

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


export COMPLINE
export GLOBAL_SETTINGS
