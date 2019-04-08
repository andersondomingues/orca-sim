APP_DIR = $(SRC_DIR)/../applications/test_counters_memory

test_counters_memory.o:
	$(CC) $(CFLAGS) -c $(APP_DIR)/test_counters_memory.c  -o test_counters_memory.o
