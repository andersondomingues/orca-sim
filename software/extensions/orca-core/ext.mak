# Do not modify the lines below
ORCA_CORE_NAME := orca-core
ORCA_CORE_DIR := $(HFOS_DIR)/../extensions/$(ORCA_CORE_NAME)
ORCA_CORE_SRC := $(ORCA_CORE_DIR)/src
ORCA_CORE_INC := $(ORCA_CORE_DIR)/include
ORCA_CORE_LIB := ext-$(ORCA_CORE_NAME).a 

# Update these lines with your source code
ORCA_CORE_OBJS := \
	orca-core.o
	
#orca-hardware-counters.o \
#	orca-systime.o

#pack everithing in a single lib
$(ORCA_CORE_LIB) : $(ORCA_CORE_OBJS)
	ar rcs $(ORCA_CORE_LIB) $(ORCA_CORE_OBJS) 

#compile each individual object file
%.o: $(ORCA_CORE_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(ORCA_CORE_INC)

#check whether .h are up to date
$(ORCA_CORE_SRC)/%.c: $(ORCA_CORE_INC)/%.h
