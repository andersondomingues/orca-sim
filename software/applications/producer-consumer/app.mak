#this script compiles object code for the 
#producer-consumer application

#this definition is used once
APP_SRC_DIR = $(SRC_DIR)/../applications/producer-consumer

#recipie for the object code. This object file will
#be linked within the image of the system by the top
#makefile
producer-consumer.o:
	$(CC) $(CFLAGS) -c \
	$(APP_SRC_DIR)/producer.c \
	$(APP_SRC_DIR)/consumer.c


