MORM_SP_DIR = $(SRC_DIR)/../applications/morm_sp

morm_sp.o:
	$(CC) $(CFLAGS) -c $(MORM_SP_DIR)/morm_sp.c -o morm_sp.o
