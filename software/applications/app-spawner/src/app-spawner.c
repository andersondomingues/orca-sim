#include <hellfire.h>
#include <noc.h>

#include <app-spawner.h>

//maximum packet length for transfering task info
#define MESSAGE_PAYLOAD_SIZE 200

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
void app_spawner_spawn(int8_t* buffer, uint32_t buffer_len){

	char task_name[40];

	uint32_t stack_size, funcptr;
	uint16_t period, capacity, deadline;

	uint32_t* buffer_32 = (uint32_t*) buffer;
	uint16_t* buffer_16 = (uint16_t*) buffer;

	//unpack data
	funcptr = buffer_32[0];
	stack_size = buffer_32[1];

	period   = buffer_16[4];
	capacity = buffer_16[5];
	deadline = buffer_16[6];

	strcpy(task_name, (char*)&(buffer_32[5]));

	printf("%d: spawning %s 0x%x (%d/%d/%d) \n",
		hf_selfid(), task_name, funcptr, period, capacity, deadline);

	hf_spawn((void*)funcptr, period, capacity, deadline, task_name, stack_size);
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
				// printf("SPAWNER: task spawn requested from %d:%d (requester), task size=%d\n", cpu, task, size);
				app_spawner_spawn(buf, size);
			}else{
				printf("hf_recv(): error %d\n", val);
			}
		}

	}
}