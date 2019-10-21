# Do not modify the lines below
APP_PROD_CONS_PS_NAME  := producer-consumer-pubsub
APP_PROD_CONS_PS_DIR   := $(SRC_DIR)/../applications/$(APP_PROD_CONS_PS_NAME)
APP_PROD_CONS_PS_SRC   := $(APP_PROD_CONS_PS_DIR)/src
APP_PROD_CONS_PS_INC   := $(APP_PROD_CONS_PS_DIR)/include
APP_PROD_CONS_PS_LIB   := app-$(APP_PROD_CONS_PS_NAME).a 

# Update these lines with your source code
APP_PROD_CONS_PS_OBJS := \
	producer-pubsub.o \
	consumer-pubsub.o

#pack everithing in a single lib
$(APP_PROD_CONS_PS_LIB) : $(APP_PROD_CONS_PS_OBJS)
	ar rcs $(APP_PROD_CONS_PS_LIB) $(APP_PROD_CONS_PS_OBJS) 

#compile each individual object file
%.o: $(APP_PROD_CONS_PS_SRC)/%.c
	$(CC) $(CFLAGS) -c -o $@ $< -I$(APP_PROD_CONS_PS_INC)

#check whether .h are up to date
$(APP_PROD_CONS_SRC)/%.c: $(APP_PROD_CONS_PS_INC)/%.h
