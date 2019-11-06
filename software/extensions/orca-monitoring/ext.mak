# Do not modify the lines below
ORCA_MONITORING_NAME := orca-monitoring
ORCA_MONITORING_DIR := $(HFOS_DIR)/../extensions/$(ORCA_MONITORING_NAME)
ORCA_MONITORING_SRC := $(ORCA_MONITORING_DIR)/src
ORCA_MONITORING_INC := $(ORCA_MONITORING_DIR)/include
ORCA_MONITORING_LIB := ext-$(ORCA_MONITORING_NAME).a 

# Update these lines with your source code
ORCA_MONITORING_OBJS := \
	orca-hardware-counters.o

#pack everithing in a single lib
$(ORCA_MONITORING_LIB) : $(ORCA_MONITORING_OBJS)
	ar rcs $(ORCA_MONITORING_LIB) $(ORCA_MONITORING_OBJS) 

#compile each individual object file
%.o: $(ORCA_MONITORING_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(ORCA_MONITORING_INC)

#check whether .h are up to date
$(ORCA_MONITORING_SRC)/%.c: $(ORCA_MONITORING_INC)/%.h
