APP_DIR = $(SRC_DIR)/../applications/$(APP)

app: kernel
	$(CC) $(CFLAGS) -c $(APP_DIR)/noc_test4_sender.c 
	$(CC) $(CFLAGS) -c $(APP_DIR)/noc_test4_receiver.c
	
