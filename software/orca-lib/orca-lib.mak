ORCALIB_DIR := $(HFOS_DIR)/../orca-lib

ORCALIB_SRC := $(ORCALIB_DIR)/src
ORCALIB_INC := $(ORCALIB_DIR)/include

ORCALIB_DEPS := \
	$(ORCALIB_SRC)/orca-lib.c \
	$(ORCALIB_SRC)/orca-hardware-counters.c

orca-lib.o: $(ORCALIB_DEPS)
	$(CC) $(CFLAGS) -c $(ORCALIB_DEPS) -I$(ORCALIB_INC)
