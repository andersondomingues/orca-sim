ORCA_STATIC_MAPPING_DIR = $(SRC_DIR)/../orca-static-mapping

orca-static-mapping.o: $(ORCA_STATIC_MAPPING_DIR)/mapper.c
	$(CC) $(CFLAGS) -c $(ORCA_STATIC_MAPPING_DIR)/mapper.c 
