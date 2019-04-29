APP_DIR = $(SRC_DIR)/../applications/morm_sp

morm_sp.o:
	$(CC) $(CFLAGS) -c $(APP_DIR)/morm_sp.c -o morm_sp.o
