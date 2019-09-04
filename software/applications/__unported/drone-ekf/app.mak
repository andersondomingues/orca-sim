DRONE_EKF_DIR = $(SRC_DIR)/../applications/drone-ekf

drone-ekf.o:
	$(CC) $(CFLAGS) -c $(DRONE_EKF_DIR)/drone-ekf.c -o drone-ekf.o
