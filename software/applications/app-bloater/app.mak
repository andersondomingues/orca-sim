# Do not modify the lines below
APP_BLOATER_NAME  := app-bloater
APP_BLOATER_DIR   := ./applications/$(APP_BLOATER_NAME)
APP_BLOATER_SRC   := $(APP_BLOATER_DIR)/src
APP_BLOATER_INC   := $(APP_BLOATER_DIR)/include
APP_BLOATER_LIB   := app-$(APP_BLOATER_NAME).a

INC_DIRS += -I$(APP_BLOATER_INC)

CFLAGS += 

# Update these lines with your source code
APP_BLOATER_SRCS := $(wildcard $(APP_BLOATER_SRC)/*.c)
APP_BLOATER_OBJS :=  $(APP_BLOATER_SRCS:.c=.o)

$(APP_BLOATER_LIB) : $(APP_BLOATER_OBJS)
	$(Q)$(AR) rcs $(APP_BLOATER_LIB) $(APP_BLOATER_OBJS) 
