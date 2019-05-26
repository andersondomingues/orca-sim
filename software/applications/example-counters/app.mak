EXAMPLE_COUNTERS_DIR = $(SRC_DIR)/../applications/example-counters

example-counters.o:
	$(CC) $(CFLAGS) -c $(EXAMPLE_COUNTERS_DIR)/example-counters.c -o examples-counters.o
