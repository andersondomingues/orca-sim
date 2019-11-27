# Do not modify the lines below
APP_SPAWNER_NAME := app-spawner
APP_SPAWNER_DIR  := $(SRC_DIR)/../applications/$(APP_SPAWNER_NAME)
APP_SPAWNER_SRC  := $(APP_SPAWNER_DIR)/src
APP_SPAWNER_INC  := $(APP_SPAWNER_DIR)/include
APP_SPAWNER_LIB  := app-$(APP_SPAWNER_NAME).a 

# Update these lines with your source code
APP_SPAWNER_OBJS := \
	app-spawner.o

#pack everithing in a single lib
$(APP_SPAWNER_LIB) : $(APP_SPAWNER_OBJS)
	ar rcs $(APP_SPAWNER_LIB) $(APP_SPAWNER_OBJS) 

#compile each individual object file
%.o: $(APP_SPAWNER_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(APP_SPAWNER_INC)

#check whether .h are up to date
$(APP_SPAWNER_SRC)/%.c: $(APP_SPAWNER_INC)/%.h
