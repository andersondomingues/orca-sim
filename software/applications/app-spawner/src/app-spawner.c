#include <hellfire.h>
#include <noc.h>

#include <app-spawner.h>

//maximum packet length for transfering task info
#define MESSAGE_PAYLOAD_SIZE 200

void app_spawner_spawn(int8_t* buffer, uint32_t buffer_len){

	// uint32_t* buffer_32 = (uint32_t*) buffer;
	// uint16_t* buffer_16 = (uint16_t*) buffer;

	// //extract task name from package
	// printf("task name: \n");

	// //extract stack size
	// printf("stack size: %d\n", buffer_32[1]);

	// //locate address of tasks func
	// printf("func addr: %d\n", buffer_32[0]);
	// // void (*fun_ptr)(void) = (void (*)(void))task_base_ptr;

	// //real time params
	// printf("rt: period=%d, capacity=%d, deadline=%d\n", 
	// 	buffer_16[4], buffer_16[5], buffer_16[6]);

	// hf_spawn(fun_ptr, 0, 0, 0, "migrated_task_001", 4096);

	//copy contents from the buffer to the new location
	//memcpy(task_base_ptr, task_code, task_size);

	//TODO: adjust jumps and other relevant instructions to 
	//reflect current application base pointer
		
	//spawn the task
	//TODO: we assume that the entry-point of the task
	//is located at the first instruction of the executable
	//code. It may change given that some tasks may have 
	//more than one function and the order of functions is 
	//arbitrarily changed by the compiler.
	//TODO: task name and period must be parameterizable.
		
	
	//TODO: restore task context
}

/** this task spawns applications once it receive the proper
 * configuration. Configuration include the name of the 
 * task to be spawned, stack size, and real-time parameters */
void app_spawner(void)
{
	int8_t buf[MESSAGE_PAYLOAD_SIZE];

	uint16_t cpu, task, size;
	int32_t val, i;

	if (hf_comm_create(hf_selfid(), 4000, 0))
		panic(0xff);

	delay_ms(1);

	//keep probing for new requests
	while (1){

		//printf("spawner time...\n");

		//periodically checks for messages
		i = hf_recvprobe();

		//read received packet
		if (i >= 0) {

			val = hf_recv(&cpu, &task, buf, &size, i);

			//check whether the received parameters are valid
			if (!val){
				printf("SPAWNER: task spawn requested from %d:%d (requester), task size=%d\n", cpu, task, size);
				// app_spawner_spawn(buf, size);
			}else{
				printf("hf_recv(): error %d\n", val);
			}
		}
	}
}