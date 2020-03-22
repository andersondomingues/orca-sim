# Do not modify the lines below
ORCA_CORE_NAME  := orca-core
ORCA_CORE_DIR   := ./extensions/$(ORCA_CORE_NAME)
ORCA_CORE_SRC   := $(ORCA_CORE_DIR)/src
ORCA_CORE_INC   := $(ORCA_CORE_DIR)/include
ORCA_CORE_LIB   := ext-$(ORCA_CORE_NAME).a

INC_DIRS += -I$(ORCA_CORE_INC)

CFLAGS += 

# Update these lines with your source code
ORCA_CORE_SRCS := $(wildcard $(ORCA_CORE_SRC)/*.c)
ORCA_CORE_OBJS :=  $(ORCA_CORE_SRCS:.c=.o)

$(ORCA_CORE_LIB) : $(ORCA_CORE_OBJS)
	$(Q)$(AR) rcs $(ORCA_CORE_LIB) $(ORCA_CORE_OBJS) 
