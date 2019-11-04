# Do not modify the lines below
APP_NOCTEST4_NAME  := noc_test4
APP_NOCTEST4_DIR   := $(SRC_DIR)/../applications/$(APP_NOCTEST4_NAME)
APP_NOCTEST4_SRC   := $(APP_NOCTEST4_DIR)/src
APP_NOCTEST4_INC   := $(APP_NOCTEST4_DIR)/include
APP_NOCTEST4_LIB   := app-$(APP_NOCTEST4_NAME).a 

# Update these lines with your source code
APP_NOCTEST4_OBJS := \
	noc_test4_sender.o \
	noc_test4_receiver.o

#pack everithing in a single lib
$(APP_NOCTEST4_LIB) : $(APP_NOCTEST4_OBJS)
	ar rcs $(APP_NOCTEST4_LIB) $(APP_NOCTEST4_OBJS) 

#compile each individual object file
%.o: $(APP_NOCTEST4_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(APP_NOCTEST4_INC)

#check whether .h are up to date
$(APP_NOCTEST4_SRC)/%.c: $(APP_NOCTEST4_INC)/%.h
