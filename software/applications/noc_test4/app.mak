APP_DIR = $(SRC_DIR)/../applications/$(APP)

app: kernel
	$(CC) $(CFLAGS) \
		$(APP_DIR)/noc_test4.c 
