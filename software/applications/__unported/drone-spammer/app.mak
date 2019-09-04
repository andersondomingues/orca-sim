DRONE_SPAMMER_DIR = $(SRC_DIR)/../applications/drone-spammer

drone-spammer.o:
	$(CC) $(CFLAGS) -c $(DRONE_SPAMMER_DIR)/drone-spammer.c -o drone-spammer.o
