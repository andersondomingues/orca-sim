DRONE_PID_DIR = $(SRC_DIR)/../applications/drone-pid

drone-pid.o:
	$(CC) $(CFLAGS) -c $(DRONE_PID_DIR)/drone-pid.c -o drone-pid.o
