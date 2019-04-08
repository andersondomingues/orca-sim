LAUNCHER_DIR = $(SRC_DIR)/../orca-mapper

.PHONY: orca-mapper.o
orca-mapper.o:
	$(CC) $(CFLAGS) -c $(LAUNCHER_DIR)/mapper.c 
