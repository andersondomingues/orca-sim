#place here only the hardware configuration parameters that are shared with its drivers

# Width (x-axis coordinate) of the network-on-chip. Cannot be zero,
# otherwise simulation won't compile.
ORCA_NOC_HEIGHT := 3
# Width (y-axis coordinate) of the network-on-chip. Cannot be zero,
# otherwise simulation won't compile.
ORCA_NOC_WIDTH  := 2

export CFLAGS += -DNOC_INTERCONNECT -DNOC_PACKET_SIZE=64 -DNOC_PACKET_SLOTS=64 \
	    -DNOC_WIDTH=$(ORCA_NOC_WIDTH) -DNOC_HEIGHT=$(ORCA_NOC_HEIGHT)

