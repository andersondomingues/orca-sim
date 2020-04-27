#place here only the hardware configuration parameters that are shared with its drivers

# Width (x-axis coordinate) of the network-on-chip. Cannot be zero,
# otherwise simulation won't compile.
export ORCA_NOC_HEIGHT := 3
# Width (y-axis coordinate) of the network-on-chip. Cannot be zero,
# otherwise simulation won't compile.
export ORCA_NOC_WIDTH  := 2

export CXXFLAGS += -DNOC_INTERCONNECT -DNOC_PACKET_SIZE=64 -DNOC_PACKET_SLOTS=64 \
	-DORCA_NOC_HEIGHT=$(ORCA_NOC_HEIGHT) -DORCA_NOC_WIDTH=$(ORCA_NOC_WIDTH) \

export CFLAGS += CXXFLAGS

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

#router parameters
ifeq ($(ROUTER_ENABLE_COUNTERS), YES)
	COMPLINE := $(COMPLINE) -DROUTER_ENABLE_COUNTERS
endif
ifeq ($(ROUTER_PORT_CONNECTED_CHECKING), YES)
	COMPLINE := $(COMPLINE) -DROUTER_PORT_CONNECTED_CHECKING
endif

export COMPLINE
