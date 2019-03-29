APP_DIR = $(SRC_DIR)/../applications/test_task_no_dependency

test_task_no_dependency.o:
	$(CC) $(CFLAGS) -c $(APP_DIR)/test_task_no_dependency.c  -o test_task_no_dependency.o
