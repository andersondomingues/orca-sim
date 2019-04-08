APP_DIR = $(SRC_DIR)/../applications/noc_test4

noc_test4_sender.o:
	$(CC) $(CFLAGS) -c $(APP_DIR)/noc_test4_sender.c  -o noc_test4_sender.o

noc_test4_receiver.o:
	$(CC) $(CFLAGS) -c $(APP_DIR)/noc_test4_receiver.c -o noc_test4_receiver.o
	
