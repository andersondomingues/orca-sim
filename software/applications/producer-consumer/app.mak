# Do not modify the lines below
APP_PROD_CONS_NAME  := producer-consumer
APP_PROD_CONS_DIR   := ./applications/$(APP_PROD_CONS_NAME)
APP_PROD_CONS_SRC   := $(APP_PROD_CONS_DIR)/src
APP_PROD_CONS_INC   := $(APP_PROD_CONS_DIR)/include
APP_PROD_CONS_LIB   := app-$(APP_PROD_CONS_NAME).a 

INC_DIRS += -I$(APP_PROD_CONS_INC)

CFLAGS += 

# Update these lines with your source code
APP_PROD_CONS_SRCS := $(wildcard $(APP_PROD_CONS_SRC)/*.c)
APP_PROD_CONS_OBJS :=  $(APP_PROD_CONS_SRCS:.c=.o)

$(APP_PROD_CONS_LIB) : $(APP_PROD_CONS_OBJS)
	$(Q)$(AR) rcs $(APP_PROD_CONS_LIB) $(APP_PROD_CONS_OBJS) 
