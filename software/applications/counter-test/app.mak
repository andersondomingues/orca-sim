# Do not modify the lines below
APP_COUNTER_TEST_NAME  := counter-test
APP_COUNTER_TEST_DIR   := $(SRC_DIR)/../applications/$(APP_COUNTER_TEST_NAME)
APP_COUNTER_TEST_SRC   := $(APP_COUNTER_TEST_DIR)/src
APP_COUNTER_TEST_INC   := $(APP_COUNTER_TEST_DIR)/include
APP_COUNTER_TEST_LIB   := app-$(APP_COUNTER_TEST_NAME).a 

# Update these lines with your source code
APP_COUNTER_TEST_OBJS := \
	counter-test.o

#pack everithing in a single lib
$(APP_COUNTER_TEST_LIB) : $(APP_COUNTER_TEST_OBJS)
	ar rcs $(APP_COUNTER_TEST_LIB) $(APP_COUNTER_TEST_OBJS) 

#compile each individual object file
%.o: $(APP_COUNTER_TEST_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(APP_COUNTER_TEST_INC)

#check whether .h are up to date
$(APP_COUNTER_TEST_SRC)/%.c: $(APP_COUNTER_TEST_INC)/%.h
