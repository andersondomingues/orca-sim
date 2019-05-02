APP_EXAMPLE_COUNTERS_DIR := $(SRC_DIR)/../applications/example_counters

example_counters.o:
	$(CC) $(CFLAGS) -c $(APP_EXAMPLE_COUNTERS_DIR)/example_counters.c  -o example_counters.o
