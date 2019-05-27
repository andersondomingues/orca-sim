#this script compiles object code for the 
#example-echo-print application

#this definition is used once
APP_EXAMPLE_ECHO_PRINT_DIR = $(SRC_DIR)/../applications/example-echo-print

#recipie for the object code. This object file will
#be linked within the image of the system by the top
#makefile
example-echo-print.o:
	$(CC) $(CFLAGS) -c \
	$(APP_EXAMPLE_ECHO_PRINT_DIR)/example-echo-print.c


