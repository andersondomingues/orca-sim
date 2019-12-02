# Do not modify the lines below
APP_DEADLINE_MONITOR_NAME := deadline-monitor
APP_DEADLINE_MONITOR_DIR  := $(SRC_DIR)/../applications/$(APP_DEADLINE_MONITOR_NAME)
APP_DEADLINE_MONITOR_SRC  := $(APP_DEADLINE_MONITOR_DIR)/src
APP_DEADLINE_MONITOR_INC  := $(APP_DEADLINE_MONITOR_DIR)/include
APP_DEADLINE_MONITOR_LIB  := app-$(APP_DEADLINE_MONITOR_NAME).a 

# Update these lines with your source code
APP_DEADLINE_MONITOR_OBJS := \
	deadline-monitor.o

#pack everithing in a single lib
$(APP_DEADLINE_MONITOR_LIB) : $(APP_DEADLINE_MONITOR_OBJS)
	ar rcs $(APP_DEADLINE_MONITOR_LIB) $(APP_DEADLINE_MONITOR_OBJS) 

#compile each individual object file
%.o: $(APP_DEADLINE_MONITOR_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(APP_DEADLINE_MONITOR_INC)

#check whether .h are up to date
$(APP_DEADLINE_MONITOR_SRC)/%.c: $(APP_DEADLINE_MONITOR_INC)/%.h
