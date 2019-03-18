LAUNCHER_DIR = $(SRC_DIR)/../orca-launcher

orca-launcher: kernel
	$(CC) $(CFLAGS) -c $(LAUNCHER_DIR)/launcher.c 
