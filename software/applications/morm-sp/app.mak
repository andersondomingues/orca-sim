MORM_SP_DIR = $(SRC_DIR)/../applications/morm-sp

morm-sp.o:
	$(CC) $(CFLAGS) -c $(MORM_SP_DIR)/morm-sp.c -o morm-sp.o
