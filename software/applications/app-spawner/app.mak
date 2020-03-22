# Do not modify the lines below
APP_SPAWNER_NAME  := app-spawner
APP_SPAWNER_DIR   := ./applications/$(APP_SPAWNER_NAME)
APP_SPAWNER_SRC   := $(APP_SPAWNER_DIR)/src
APP_SPAWNER_INC   := $(APP_SPAWNER_DIR)/include
APP_SPAWNER_LIB   := app-$(APP_SPAWNER_NAME).a

INC_DIRS += -I$(APP_SPAWNER_INC)

CFLAGS += 

# Update these lines with your source code
APP_SPAWNER_SRCS := $(wildcard $(APP_SPAWNER_SRC)/*.c)
APP_SPAWNER_OBJS :=  $(APP_SPAWNER_SRCS:.c=.o)

$(APP_SPAWNER_LIB) : $(APP_SPAWNER_OBJS)
	$(Q)$(AR) rcs $(APP_SPAWNER_LIB) $(APP_SPAWNER_OBJS) 

