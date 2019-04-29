#include <hellfire.h>
#include <noc.h>

#define MAX_TASK_SIZE 2500


/**
 * Allocate memory for some task and spawn it.
 * @param task_code Buffer containing the executable code for the task
 * @param task_size The length of the task in bytes
*/
void mapper_map_task(int8_t* task_code, uint32_t task_size){

	//alloc memory for placing the task
	int8_t* task_base_ptr;
	task_base_ptr = (int8_t*)hf_malloc(task_size);

	//copy contents from the buffer to the new location
	memcpy(task_base_ptr, task_code, task_size);

	//TODO: adjust jumps and other relevant instructions to 
	//reflect current application base pointer
		
	//spawn the task
	//TODO: we assume that the entry-point of the task
	//is located at the first instruction of the executable
	//code. It may change given that some tasks may have 
	//more than one function and the order of functions is 
	//arbitrarily changed by the compiler.
	//TODO: task name and period must be parameterizable.
	void (*fun_ptr)(void) = (void (*)(void))task_base_ptr;
	hf_spawn(fun_ptr, 0, 0, 0, "migrated_task_001", 4096);
	
	//TODO: restore task context
}

/**
 * TASK Task Mapper
 * This task is capable of transfering tasks states and 
 * start those tasks remotely, similarly to task migration
 * but migrating only the state
*/
void mapper_listener(void)
{
	int8_t buf[MAX_TASK_SIZE];

	uint16_t cpu, task, size;
	int32_t val, i;

	if (hf_comm_create(hf_selfid(), 5000, 0))
		panic(0xff);

	//keep probing for new requests
	while (1){
	
		//periodically checks for messages
		i = hf_recvprobe();

		//read received packet	
		if (i >= 0) {
	
			val = hf_recv(&cpu, &task, buf, &size, i);

			if (!val){
				printf("SPAWNER: task migration request from cpu=%d, task=%d (requester), task size=%d\n", cpu, task, size);
				mapper_map_task(buf, size);
			}else{
				printf("hf_recv(): error %d\n", val);
			}
		}
	}
}

/* entry-point */
void app_main(void)
{
	//the spawner is instantiated as a best-effort task to 
	//nagate any impact on currently executing tasks
	//if(hf_cpuid() == 2)
	//	hf_spawn(mapper_listener, 0, 0, 0, "mapper", 4096);
	
	hf_spawn(morm_sp_task, 0, 0, 0, "test_counters", 4096);
	
	//if cluster_master then spawn lm
	//if global_master then spawn gp
	
	//printf("task_id => %d", hf_id("idle"));	
}
