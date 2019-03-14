SPAWNER_DIR = $(SRC_DIR)/../orca-spawner

spawner: kernel
	$(CC) $(CFLAGS) -c $(SPAWNER_DIR)/spawner.c 
