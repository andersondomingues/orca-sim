# Do not modify the lines below
ORCA_PUBSUB_DIR := $(HFOS_DIR)/../extensions/orca-pubsub
ORCA_PUBSUB_SRC := $(ORCA_PUBSUB_DIR)/src
ORCA_PUBSUB_INC := $(ORCA_PUBSUB_DIR)/include

# Update these lines with your source code
ORCA_PUBSUB_DPS := \
	$(ORCA_PUBSUB_SRC)/pubsub-broker-task.c \
	$(ORCA_PUBSUB_SRC)/pubsub-client.c

# Do not modify this recipie
orca-pubsub.o : $(ORCA_PUBSUB_DPS)
	$(CC) $(CFLAGS) -c $(ORCA_PUBSUB_DPS) -I$(ORCA_PUBSUB_INC) 