EXAMPLE_SYSTIME_DIR = $(SRC_DIR)/../applications/example-systime

example-systime.o:
	$(CC) $(CFLAGS) -c $(EXAMPLE_SYSTIME_DIR)/example-systime.c -o example-systime.o
