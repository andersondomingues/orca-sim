# Do not modify the lines below
APP_PROD_CONS_NAME  := producer-consumer
APP_PROD_CONS_DIR   := $(SRC_DIR)/../applications/$(APP_PROD_CONS_NAME)
APP_PROD_CONS_SRC   := $(APP_PROD_CONS_DIR)/src
APP_PROD_CONS_INC   := $(APP_PROD_CONS_DIR)/include
APP_PROD_CONS_LIB   := app-$(APP_PROD_CONS_NAME).a 

# Update these lines with your source code
APP_PROD_CONS_OBJS := \
	producer.o \
	consumer.o

#pack everithing in a single lib
$(APP_PROD_CONS_LIB) : $(APP_PROD_CONS_OBJS)
	ar rcs $(APP_PROD_CONS_LIB) $(APP_PROD_CONS_OBJS) 

#compile each individual object file
%.o: $(APP_PROD_CONS_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(APP_PROD_CONS_INC)

#check whether .h are up to date
$(APP_PROD_CONS_SRC)/%.c: $(APP_PROD_CONS_INC)/%.h
