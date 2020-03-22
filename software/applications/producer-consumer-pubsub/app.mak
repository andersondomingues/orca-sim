# Do not modify the lines below
APP_PROD_CONS_PS_NAME  := producer-consumer-pubsub
APP_PROD_CONS_PS_DIR   := ./applications/$(APP_PROD_CONS_PS_NAME)
APP_PROD_CONS_PS_SRC   := $(APP_PROD_CONS_PS_DIR)/src
APP_PROD_CONS_PS_INC   := $(APP_PROD_CONS_PS_DIR)/include
APP_PROD_CONS_PS_LIB   := app-$(APP_PROD_CONS_PS_NAME).a 

INC_DIRS += -I$(APP_PROD_CONS_PS_INC)

CFLAGS += 

# Update these lines with your source code
APP_PROD_CONS_PS_SRCS := $(wildcard $(APP_PROD_CONS_PS_SRC)/*.c)
APP_PROD_CONS_PS_OBJS :=  $(APP_PROD_CONS_PS_SRCS:.c=.o)

$(APP_PROD_CONS_PS_LIB) : $(APP_PROD_CONS_PS_OBJS)
	$(Q)$(AR) rcs $(APP_PROD_CONS_PS_LIB) $(APP_PROD_CONS_PS_OBJS) 
