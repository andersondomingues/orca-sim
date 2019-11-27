# Do not modify the lines below
APP_BLOATER_NAME := app-bloater
APP_BLOATER_DIR  := $(SRC_DIR)/../applications/$(APP_BLOATER_NAME)
APP_BLOATER_SRC  := $(APP_BLOATER_DIR)/src
APP_BLOATER_INC  := $(APP_BLOATER_DIR)/include
APP_BLOATER_LIB  := app-$(APP_BLOATER_NAME).a 

# Update these lines with your source code
APP_BLOATER_OBJS := \
	app-bloater.o

#pack everithing in a single lib
$(APP_BLOATER_LIB) : $(APP_BLOATER_OBJS)
	ar rcs $(APP_BLOATER_LIB) $(APP_BLOATER_OBJS) 

#compile each individual object file
%.o: $(APP_BLOATER_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(APP_BLOATER_INC)

#check whether .h are up to date
$(APP_BLOATER_SRC)/%.c: $(APP_BLOATER_INC)/%.h
