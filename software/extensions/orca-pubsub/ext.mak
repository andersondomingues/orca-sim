# Do not modify the lines below
ORCA_PUBSUB_NAME := orca-pubsub
ORCA_PUBSUB_DIR := $(HFOS_DIR)/../extensions/$(ORCA_PUBSUB_NAME)
ORCA_PUBSUB_SRC := $(ORCA_PUBSUB_DIR)/src
ORCA_PUBSUB_INC := $(ORCA_PUBSUB_DIR)/include
ORCA_PUBSUB_LIB := ext-$(ORCA_PUBSUB_NAME).a 

# Update these lines with your source code
ORCA_PUBSUB_OBJS := \
	pubsub-broker.o \
	pubsub-publisher.o \
	pubsub-subscriber.o \
	pubsub-shared.o 

# Do not modify this recipe
$(ORCA_PUBSUB_LIB) : $(ORCA_PUBSUB_OBJS)
	ar rcs $(ORCA_PUBSUB_LIB) $(ORCA_PUBSUB_OBJS) 

#compile each individual object file
%.o: $(ORCA_PUBSUB_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(ORCA_PUBSUB_INC)

#check whether .h are up to date
$(ORCA_PUBSUB_SRC)/%.c: $(ORCA_PUBSUB_INC)/%.h

	