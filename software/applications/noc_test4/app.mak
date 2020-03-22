# Do not modify the lines below
APP_NOCTEST4_NAME  := noc_test4
APP_NOCTEST4_DIR   := ./applications/$(APP_NOCTEST4_NAME)
APP_NOCTEST4_SRC   := $(APP_NOCTEST4_DIR)/src
APP_NOCTEST4_INC   := $(APP_NOCTEST4_DIR)/include
APP_NOCTEST4_LIB   := app-$(APP_NOCTEST4_NAME).a 

INC_DIRS += -I$(APP_NOCTEST4_INC)

CFLAGS += 

# Update these lines with your source code
APP_NOCTEST4_SRCS := $(wildcard $(APP_NOCTEST4_SRC)/*.c)
APP_NOCTEST4_OBJS :=  $(APP_NOCTEST4_SRCS:.c=.o)

$(APP_NOCTEST4_LIB) : $(APP_NOCTEST4_OBJS)
	$(Q)$(AR) rcs $(APP_NOCTEST4_LIB) $(APP_NOCTEST4_OBJS) 