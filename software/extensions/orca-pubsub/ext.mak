# Do not modify the lines below
ORCA_PUBSUB_NAME  := orca-pubsub
ORCA_PUBSUB_DIR   := ./extensions/$(ORCA_PUBSUB_NAME)
ORCA_PUBSUB_SRC   := $(ORCA_PUBSUB_DIR)/src
ORCA_PUBSUB_INC   := $(ORCA_PUBSUB_DIR)/include
ORCA_PUBSUB_LIB   := ext-$(ORCA_PUBSUB_NAME).a

INC_DIRS += -I$(ORCA_PUBSUB_INC)

CFLAGS += 

# Update these lines with your source code
ORCA_PUBSUB_SRCS := $(wildcard $(ORCA_PUBSUB_SRC)/*.c)
ORCA_PUBSUB_OBJS :=  $(ORCA_PUBSUB_SRCS:.c=.o)

$(ORCA_PUBSUB_LIB) : $(ORCA_PUBSUB_OBJS)
	$(Q)$(AR) rcs $(ORCA_PUBSUB_LIB) $(ORCA_PUBSUB_OBJS) 
