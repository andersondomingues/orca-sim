# Do not modify the lines below
ORCA_CORE_DIR := $(HFOS_DIR)/../extensions/orca-core
ORCA_CORE_SRC := $(ORCA_CORE_DIR)/src
ORCA_CORE_INC := $(ORCA_CORE_DIR)/include

# Update these lines with your source code
ORCA_CORE_DPS := \
	$(ORCA_CORE_SRC)/orca-core.c \
	$(ORCA_CORE_SRC)/orca-hardware-counters.c \
	$(ORCA_CORE_SRC)/orca-systime.c

# Do not modify this recipie
orca-core.o : $(ORCA_CORE_DPS)
	$(CC) $(CFLAGS) -c $(ORCA_CORE_DPS) -I$(ORCA_CORE_INC) 