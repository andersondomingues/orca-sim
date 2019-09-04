DRONE_PID_DIR = $(SRC_DIR)/../applications/drone-pid

drone-pid.o:
	$(CPP) $(CFLAGS) -c $(DRONE_PID_DIR)/drone-pid.cpp -o drone-pid.o -DCPP
