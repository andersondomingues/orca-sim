#include <hellfire.h>
#include <noc.h>

#include <app-bloater.h>

#define BLOAT_DELAY 20
#define BUFFER_SIZE 96

#define BLOAT_INITIAL_DELAY 100
#define BLOAT_PERIOD_DELAY  2000

/* BLOAT PACKET INFO
	-> uint32_t func_addr
	-> uint32_t stack_size
	-> uint16_t rt_period
	-> uint16_t rt_capacity
	-> uint16_t rt_deadline
	-> uint16_t ___padding0
	-> uint32_t name_len
	-> char* name_data */

//this task does nothing
void bloat_idle_task(){

	for (;;){
		printf("%d: bloat -.-\n", hf_selfid());
	}
}

//create a bloat package and send it to the target cpu
void bloat(uint32_t funcptr, uint32_t stacksize, uint16_t cpu, uint16_t port, uint16_t task_counter){

	int8_t buffer[BUFFER_SIZE];
	char task_name[40], tmp_str[40];

	uint16_t period = 10, capacity = 1, deadline = 10;

	uint32_t* buffer_32 = (uint32_t*) buffer;
	uint16_t* buffer_16 = (uint16_t*) buffer;

	//add an unique name to the task to be spawned
	strcpy(task_name, "bloat_");
	itoa(task_counter, tmp_str, 10);
	strcat(task_name, tmp_str);
	// printf("task name: %s\n", task_name);

	//extract stack size
	// printf("stack size: %d bytes\n", stacksize);

	//locate address of tasks func
	// printf("func addr: 0x%x\n", funcptr);

	//real time params (fixed)
	// printf("rt: period=%d, capacity=%d, deadline=%d\n", 10, 1, 1);
	
    // |------- 32 bits -------|
	// _________________________
	//         functptr
	// _________________________
	//         stacksize
	// _________________________
    //   period    |   capacity
	// ____________|____________
	//   deadline  |   padding
	// ____________|____________
	//         name length      
	// _________________________
	//        name data (...)
	// _________________________

	//fill the packet 
	buffer_32[0] = funcptr;            //funcptr
	buffer_32[1] = stacksize;          //stack size
	
	buffer_16[4] = period;             //rt_period
	buffer_16[5] = capacity;           //rt_capacity
	buffer_16[6] = deadline;           //rt_deadline
	buffer_16[7] = 0x0;                //padding
	
	buffer_32[4] = strlen(task_name);  //task_name_len
	strcpy((int8_t*)&(buffer_32[5]), task_name);   //task_name 

	// hexdump(buffer, BUFFER_SIZE);
	// printf("\n");

	//send
	hf_send(cpu, port, buffer, sizeof(buffer), 1234);
	printf("%d: requester %d:%d for task %s 0x%x (%d/%d/%d) \n",
		hf_selfid(), cpu, port, task_name, funcptr, period, capacity, deadline);

}

/** this task spawns applications once it receive the proper
 * configuration. Configuration include the name of the 
 * task to be spawned, stack size, and real-time parameters */
void app_bloater(void)
{
	uint16_t target_cpu = 2, target_port = 4000, task_counter = 0;

	if (hf_comm_create(hf_selfid(), 5000, 0))
		panic(0xff);

	delay_ms(BLOAT_INITIAL_DELAY);

	//keep probing for new requests
	while (1){
	
		//spawn one task every 200ms
		delay_ms(BLOAT_PERIOD_DELAY); 

		//create bloat package and sent it
		bloat((uint32_t)(&bloat_idle_task), 1024, target_cpu, target_port, task_counter++);
		
		if(task_counter == 32){
			printf("%d: done requesting tasks\n", hf_selfid());
			break;
		}
	}
	
	while(1); //holds indefinitelly
	
}