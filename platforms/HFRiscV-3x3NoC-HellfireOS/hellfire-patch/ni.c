#include <hellfire.h>
#include <ni.h>

#define DMNI_SIZE 	        0xf0000100
#define DMNI_OP   	        0xf0000101
#define DMNI_ADDRESS        0xf0000110
#define DMNI_START 	        0xf0000111

#define DMNI_SEND_ACTIVE    0xf0001000
#define DMNI_RECEIVE_ACTIVE 0xf0001001

#define DMNI_WRITE 0
#define DMNI_READ  1

#define OP_SEND 4
#define OP_RECV 8
#define OP_NONE 0

/** TODO: It should be written elsewhere
 * @brief Sets some memory addres to some value
 * @param vaddr Address to be set
 * @param value Value to be written */
void set_mem(uint32_t vaddr, uint32_t value){
	printf("set_mem\n");
	*((uint32_t*)vaddr) = value;
}

/** TODO: It should be written elsewhere
 * @brief Read the value of some memory address
 * @param vaddr Addres to be read
 * @return The read value */
uint32_t get_mem(uint32_t vaddr){
	printf("get_mem\n");
	return *((uint32_t*)vaddr);
}

/**
 * @brief Configures DMNI to send data through the noc (HEMPS wrap)
 * @param initial_address Address of the first word of data to be sent
 * @param dmni_msg_size Total length (in words) of data to be sent */
void DMNI_send_data(unsigned int initial_address, unsigned int dmni_msg_size){

	printf("_ni_send_data\n");

	//prevents from sending data while some other 
	//sending is occuring
	uint32_t status = get_mem(DMNI_SEND_ACTIVE);
	printf("_ni_send_status %d\n", status);
	while (status);

	//program dmni to send the message
	set_mem(DMNI_SIZE, dmni_msg_size);
	set_mem(DMNI_OP, DMNI_READ);
	set_mem(DMNI_ADDRESS, initial_address);
	
	//start the dmni
	set_mem(DMNI_START, 1);
}

/**
 * @brief Configures DMNI to receive data from the noc
 * @param initial_address Address in which data should be put
 * @param dmni_msg_size Length of data to be received */
void DMNI_read_data(unsigned int initial_address, unsigned int dmni_msg_size){
	
	printf("_ni_read_data\n");
	
	//program dmni to recv the message
	set_mem(DMNI_SIZE, dmni_msg_size);
	set_mem(DMNI_OP, DMNI_READ);
	set_mem(DMNI_ADDRESS, initial_address);
	
	//start the dmni
	set_mem(DMNI_START, 1);
	
	//holds the cpu until the dmni receives all the stuff
	while (get_mem(DMNI_RECEIVE_ACTIVE));
}

//HF api (which must be implemented)
uint16_t _ni_status(void){

	printf("_ni_status\n");
	return 1; //noc is "ready"
	//return 0; //---> ?????
	
}

uint16_t _ni_read(void){
	
	printf("_ni_read\n");
	uint16_t data;
	//DMNI_read_data(&data, 2);
	return data;
}

void _ni_write(uint16_t data){
	printf("_ni_write %d\n", data);
	DMNI_send_data(&data, 2);
}