LAUNCHER_DIR = $(SRC_DIR)/../orca-launcher

.PHONY: orca-launcher.o
orca-launcher.o:
	$(CC) $(CFLAGS) -c $(LAUNCHER_DIR)/launcher.c 
