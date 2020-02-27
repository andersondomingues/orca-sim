#include <hellfire.h>
#include <noc.h>

#include <deadline-monitor.h>

#define BUFFER_SIZE 96
#define TASK_ID 2

//create a bloat package and send it to the target cpu
void respawn(uint32_t funcptr, uint32_t stacksize, uint16_t cpu, uint16_t port, uint16_t task_counter){

	int8_t buffer[BUFFER_SIZE];
	char task_name[40], tmp_str[40];

	uint16_t period = 10, capacity = 1, deadline = 2;

	uint32_t* buffer_32 = (uint32_t*) buffer;
	uint16_t* buffer_16 = (uint16_t*) buffer;

	//add an unique name to the task to be spawned
	strcpy(task_name, "consumer-pubsub");
	itoa(task_counter, tmp_str, 15);
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

	//send
	hf_send(cpu, port, buffer, sizeof(buffer), 1234);
	printf("respawned %d:%d with task %s 0x%x (%d/%d/%d) \n",
		cpu, port, task_name, funcptr, period, capacity, deadline);
		
	while(1); //prevents the function from spawning multiple tasks 
}

void deadline_monitor(){

	if (hf_comm_create(hf_selfid(), 1432, 0))
		panic(0xff);

	delay_ms(20);

	while(1){
	
		volatile int32_t dlm = hf_dlm(TASK_ID);
		printf("%d: missed deadlines=%d\n", hf_selfid(), dlm);
		
		//print the number of deadline misses for task two
		if(dlm > 2){
			printf("!! DEADLINES MISSED > 3, REALLOC !!\n");
			respawn(
				(uint32_t)(krnl_tcb[TASK_ID].ptask), 
				4096, 4, 4000, 0);
		}
	}
}