/*--------------------------------------------------------------------
* TITLE: Plasma microkernel
* AUTHOR: Cristiane Raquel Woszezenki (cristianew@inf.pucrs.br)
*         Ismael Augusto Grehs (grehs@inf.pucrs.br)
* DATE CREATED: 11/10/06
* FILENAME: kernel.c
* PROJECT: MPSoC Plasma
* DESCRIPTION:
*    Manager microkernel
*    No scheduling, no system call, no switch context
*
*--------------------------------------------------------------------*/
int total_tasks;
/**
* Atividades do gerente:
*   - ler as tarefas da memoria e enviar aos processadores destinos
*/
#include "kernel.h"
#include "../../include/services.h"
#include "../../include/plasma.h"
#include "../../include/stdlib.h"
#include "../../include/communication.h"
#include "premap.h"

//#define UART	/* Uncoment to use the UART hadware module */


/*--------------------------------------------------------------------
* GLOBALS
*--------------------------------------------------------------------*/
/* Stores the number of tasks stored in the repository */
//int total_tasks;

/* Stores the number of dynamic inserted tasks stored in the repository */
int static_tasks = 0;

/*--------------------------------------------------------------------
* Serial Communications and Debug
*--------------------------------------------------------------------*/
#ifdef UART
int putchar(int value){

   while((MemoryRead(IRQ_STATUS) & IRQ_UART_WRITE_AVAILABLE) == 0);
	   if(value==0x0A)
			   value=0x10A;
   MemoryWrite(UART_WRITE, value);

   return 0;
}

int puts(const char *string) {
	while(*string) {
		if(*string == '\n')
		putchar('\r');
		putchar(*string++);
	}

	return 0;
}

/*--------------------------------------------------------------------
* DebugMessage
*
* DESCRIPTION:
*    Receives and Echo() packet and sends the string byte a byte to the UART
*
*--------------------------------------------------------------------*/
void DebugMessage() {

	int str_part;		/* Stores 4 characteres */
	int str_size, j;
	char *c;

	/* Indicates a slave line in the output file */
	putchar('$');

	/* Reads the source processor */
	putchar(NI_Read());

	/* Reads the target task */
	NI_Read();

	/* Reads the source task */
	putchar(NI_Read());

	/* Reads the string size in characters */
	str_size = NI_Read();

	/* Reads the string and send it to the UART */
	for(;;) {

		str_part = NI_Read();	/* Receive 4 characteres */
		c = (char*)&str_part;
		j = 4;

		while(str_size-- && j--)
			putchar(*c++);

		if ( str_size >= 0)
			str_size++;
		else
			break;
	}

	/* New line in the output file */
	putchar('\n');
}

#else	/*** Using UartFile module ***/
/*-------------------------------------------------------------------------------
* puts
*
* DESCRIPTION:
*	Sends the string to the master UartFile module.
*
*--------------------------------------------------------------------------------*/
int puts(const char *string) {
	int *str_part = (int*)string;
	
while ( MemoryRead(DMA_ACTIVE) );

	for(;;) {
		MemoryWrite(UART_WRITE,*str_part);
		if ( ((char*)str_part)[0] == 0 || ((char*)str_part)[1] == 0 || ((char*)str_part)[2] == 0 || ((char*)str_part)[3] == 0)
			break;
		else
			*str_part++;
	}

	return 0;
}

/*--------------------------------------------------------------------
* DebugMessage
*
* DESCRIPTION:
*    Receives and Echo() packet and send the string 4bytes a 4bytes to the UartFile module
*
*--------------------------------------------------------------------*/
void DebugMessage() {

	int str_size, j, words_count;

	/* Indicates a slave line in the output file */
	MemoryWrite(UART_WRITE,'$');

	/* Reads the source processor */
	MemoryWrite(UART_WRITE,NI_Read());

	/* Reads the target task */
	NI_Read();

	/* Reads the source task */
	MemoryWrite(UART_WRITE,NI_Read());

	/* Reads the string size in characters */
	str_size = NI_Read();

	/* Converts the string size from bytes to 4 bytes words */
	words_count = (str_size%4 == 0) ? str_size/4 : str_size/4 + 1;

	/* Reads the string and send it to UartFile module */
	for(j=0; j<words_count; j++)
		MemoryWrite(UART_WRITE, NI_Read());

	/* New line in the output file */
	MemoryWrite(UART_WRITE,'\n');
}
#endif

/*--------------------------------------------------------------------
* ReadIncomingServices
*
* DESCRIPTION:
*    Reads incoming NoC services while the DMA is transmiting.
*
*--------------------------------------------------------------------*/
void ReadIncomingServices() {

	int service;
	int i, empty = NONE;

	int requested_task;		/* Stores a requested task id */
	int terminated_task;	/* Stores a terminated task id */
	int requesting_task;	/* Stores the id of a task which requires a TASK_REQUEST or LOCATION_REQUEST */
	int source_processor;	/* Stores the id of a processor which requires a TASK_REQUEST or LOCATION_LOCATION */

		puts("ReadIncomingServices entrei\n");

		while ( MemoryRead(DMA_ACTIVE) )
		if ( NoC_INT ) {

			/* Reads target and packet size */
			NI_Read();

			/* Reads the incoming service */
			service = NI_Read();


			switch(service) {
				case TASK_REQUEST:
		
					/* Reads the source processor */
					source_processor = NI_Read();

					/* Reads the requested task */
					requested_task = NI_Read();

					/* Reads the requesting task */
					requesting_task = NI_Read();
					
					/* Stores a TASK_REQUEST service */
					for(i=0; i<MAX_GLOBAL_TASKS*2; i++)
						if ( task_request[i].requested_task == NONE ) {
							task_request[i].requested_task = requested_task;
							task_request[i].requesting_task = requesting_task;
							task_request[i].source_processor = source_processor;
							break;
						}

					/* There is no space to store the task request */
					if ( i == MAX_GLOBAL_TASKS*2 ) {
						puts("# There is no space to store the task request.\n");
						puts("Kernel panic!!!\n");
						for(;;);
					}
				break;


				case TASK_TERMINATED:
					
					/* Reads the processor where the task has been allocated */
					NI_Read();

					/* Reads the terminated task */
					terminated_task = NI_Read();

					task_terminated[terminated_task] = terminated_task;

				break;

				case DEBUG_MESSAGE:
					
					DebugMessage();	/* Handles an Echo() */
				
				break;

				case LOCATION_REQUEST:
				
					/* Reads the source processor */
					source_processor = NI_Read();

					/* Reads the requesting task */
					requesting_task = NI_Read();

					/* Reads the requested task location */
					requested_task = NI_Read();

					/* Stores a LOCATION_REQUEST service */
					for(i=0; i<MAX_GLOBAL_TASKS*2; i++) {

						/* Searches an empty position in the location_request array */
						if ( location_request[i].requested_task == NONE ) {
							location_request[i].requested_task = requested_task;
							location_request[i].requesting_task = requesting_task;
							location_request[i].source_processor = source_processor;
							break;
						}
					}

					/* There is no space to store the location request */
					if ( i == MAX_GLOBAL_TASKS*2) {
						puts("# There is no space to store the location request.\n");
						puts("# Kernel panic!!!\n");
						for(;;);
					}

				break;
				
				default:
		    		puts("# DEFAULT desconhecido\n");
        		break;
			}
		}
		puts("ReadIncomingServices sai\n");

}


/*--------------------------------------------------------------------
* InsertTaskLoc
*
* DESCRIPTION:
*    Inserts a new task in the location table.
*
*--------------------------------------------------------------------*/
void InsertTaskLoc(int id, int proc) {

    int i;

	task_location[id].task = id;
	task_location[id].processor = proc;

}

/*--------------------------------------------------------------------
* PageUsed
*
* DESCRIPTION:
*    Decrements free_pages of processor proc.
*
*--------------------------------------------------------------------*/
void PageUsed(int proc) {

    pe_free_pages[proc >> 4][proc & 0xF]--;
    total_free_pes--;
	if(pe_type[proc >> 4][proc & 0xF] == PLASMA) free_plasmas--;
	else free_mblites--;
}

/*--------------------------------------------------------------------
* PageReleased
*
* DESCRIPTION:
*    Increments free_pages of processor proc.
*
*--------------------------------------------------------------------*/
void PageReleased(int proc) {

   pe_free_pages[proc >> 4][proc & 0xF]++;
   total_free_pes++;
   if(pe_type[proc >> 4][proc & 0xF] == PLASMA) free_plasmas++;
   else free_mblites++;
}

/*--------------------------------------------------------------------
* StaticAllocation
*
* DESCRIPTION:
*    Allocates the tasks dragged to the slave processors.
*
*--------------------------------------------------------------------*/
void StaticAllocation() {

    int i;
    TaskPackage* task;

    /* Reads the number of tasks stored in the tasks repository */
    total_tasks = *((int*)0x10000000);

    /* Points the first task header in the task repository */
    task = (TaskPackage*)0x10000004;


    /* Allocates the static tasks */
    for(i = 0; i<total_tasks; i++) {
		
		if (task[i].proc_addr != DYNAMIC) {
			
			/* Updates the task_location array */
			InsertTaskLoc(task[i].id & 0x7fffffff,task[i].proc_addr);

			/* Updates the available pages in the slave processor */
			PageUsed(task[i].proc_addr);
			
			/* Allocates the task to the slave processor */
//			TaskAllocation(task[i].proc_addr, &task[i]);
			task_pes[task->id & 0x7fffffff] = task[i].proc_addr;
			SendCtrlPacket(task[i].proc_addr, TASK_ALLOCATION, 0, 0, &task[i]);
			
			ReadIncomingServices();
			
			static_tasks++;
        }
    }
}

/*--------------------------------------------------------------------
* Initial Mapping
*
* DESCRIPTION:
*    Searches for the processor to map an initial tasks of an application.
*
*--------------------------------------------------------------------*/
int InitialMapping(int type) {

   int x, y, xo, yo, hops, pe, max_free_pes, free_pes;
   int xi, xf, yi, yf;

	pe = -1;
	hops = 2;
	max_free_pes = -1;

	for(xo=0; xo<XDIMENSION; xo++)
	{
		for(yo=0; yo<YDIMENSION; yo++)
		{
			if(pe_free_pages[xo][yo]>0 && pe_type[xo][yo] == type)
			{
				xi = xo-hops;
				xf = xo+hops;
				yi = yo-hops;
				yf = yo+hops;

				if(xi<0) xi=0;
				if(yi<0) yi=0;
				if(xf>(XDIMENSION-1)) xf=XDIMENSION-1;
				if(yf>(YDIMENSION-1)) yf=YDIMENSION-1;
				
				free_pes = 0;
				
				for(x=xi;x<=xf;x++)
				{
					for(y=yi;y<=yf;y++)
					{
						if(((abs(xo-x)+abs(yo-y))<=hops)&&(pe_free_pages[x][y]>0))
						{
							free_pes = free_pes + pe_free_pages[x][y];
						}
					}
				}
				if(free_pes>max_free_pes)
				{	
					pe = xo*16 + yo;
					max_free_pes = free_pes;
				}
			}
		}
	}
	
	if(pe == -1)
	{
		return NONE;
	}
	else
	{
		return pe;
	}

}

/*--------------------------------------------------------------------
* Main
*
* DESCRIPTION:
* 	- Iinitializes data structures.
*	- Handles/Stores the incoming NoC services.
*
*--------------------------------------------------------------------*/
int main(void){

	int i , j, cont, service, taskID=0, premapped=0;
	int proc, source_processor = 0, requesting_task = 0;
	int task_type, repository_taskID;
	int total_tasks_old = 0;
	int o;
    int there_is_mapped_tasks;

	TaskPackage* task;
	task = (TaskPackage*)0x10000004; //external memory address (Tasks repository)

	net_address = MemoryRead(NI_CONFIG); // for communications.h

	char pendent_task_request = FALSE;		/* Indicates a stored TASK_REQUEST service (task_request[]) */
	char pendent_task_terminated = FALSE;	/* Indicates a stored TASK_TERMINATED service (task_terminated[]) */
	char pendent_location_request = FALSE;	/* Indicates a stored LOCATION_REQUEST service (location_request[]) */
	char incoming_service = FALSE;			/* Indicates an incoming service (NoC) */

	pe_free_pages[MASTERADDRESS >> 4][MASTERADDRESS & 0xF] = 0;
	
    puts("# Master strikes again.\n");

	//sends tasks to the processors
	StaticAllocation();
	
	puts("# Static mapped tasks allocated.\n");
	
	if(static_tasks==0) total_tasks = 0;
	
	for(i = 0; i<total_tasks; i++) {
		
		if(task[i].id >= 0x80000000)
		{
			taskList[i].type = MBLITE;
		}
		else
		{
			taskList[i].type = PLASMA;
		}
		
		cont = 0;
		for(j = 0; j<10; j++)
		{
			if(task[i].dependences[j].task != DYNAMIC)
			{
				taskList[i].taskDep[j].task = task[i].dependences[j].task;
				taskList[i].taskDep[j].flits = task[i].dependences[j].flits;
				cont++;
			}
		}
		taskList[i].dependences = cont;
	}
	
	for(i = 0; i<total_tasks; i++) 
	{
		if (task[i].proc_addr != DYNAMIC) 
		{
			if(pe_free_pages[task[i].proc_addr >> 4][task[i].proc_addr & 0xF]>0) 
			{
				premapping(task[i].proc_addr, i);
			}
		}
	}
	
	/* Initiates the handling of incoming/stored  services */
    for(;;) {

		if ( NoC_INT || pendent_task_terminated || pendent_task_request || pendent_location_request || Task_Req) {

			if ( NoC_INT ) {
				incoming_service = TRUE;

				/* Reads target and packet size */
				NI_Read();

				/* Reads the incoming service */
				service = NI_Read();
			}
			
			else if (Task_Req) {
				incoming_service = FALSE;
				service = NEW_TASK;
				taskID	= External_Task;
				if(taskID>=total_tasks)
				{
					total_tasks_old = total_tasks;
					total_tasks = *((int*)0x10000000);
				}
			}
			
			else if ( pendent_task_terminated ) {
				incoming_service = FALSE;
				service = TASK_TERMINATED;

			}

			else if ( pendent_task_request ) {
				incoming_service = FALSE;
				service = TASK_REQUEST;
			}

			else {
				incoming_service = FALSE;
				service = LOCATION_REQUEST;
			}

			switch(service) {
				case TASK_REQUEST:

					/* Verifies if the TASK_REQUEST is incoming (NoC) or stored (task_request[]) */
					if ( incoming_service ) {

						/* Reads the source processor */
						source_processor = NI_Read();
						
						/* Reads the requested task */
						taskID = NI_Read();
						
						/* Reads the requesting task */
						requesting_task = NI_Read();


					
					}
					if ( task_location[taskID].task == taskID ) {
						
						proc = task_location[taskID].processor;
				
						for(i=0; i<MAX_GLOBAL_TASKS*2; i++)
							if ( task_request[i].requested_task == taskID && task_request[i].source_processor == source_processor && task_request[i].requesting_task == requesting_task) {
								task_request[i].requested_task = NONE;
								break;
							}
							
						/*** Sends the requested task location to the requesting task ***/

						/* Sets the requesting task processor */
//						slot.remote_addr = source_processor;

						/* Sets the service */
//						slot.service = TASK_ALLOCATED;

						/* Sets the processor where the requested task has been allocated */
//						slot.arg[0] = proc;

						/* Sets the requested task */
//						slot.arg[1] = taskID;

						/* Sets the packet size (flits) */
//						slot.pkt_size = 3 * 2;

						SendCtrlPacket(source_processor, TASK_ALLOCATED, proc, taskID, 0);

//						DMA_Send(&slot);

						/* Reads incoming NoC services while the message is transmited */
						ReadIncomingServices();


						/*** Sends the requesting task location the requested task ***/

						/* Sets the requested task processor */
//						slot.remote_addr = proc;

						/* Sets the service */
//						slot.service = TASK_ALLOCATED;

						/* Sets the processor where the requesting task is allocated */
//						slot.arg[0] = source_processor;

						/* Sets the requesting task */
//						slot.arg[1] = requesting_task;

						/* Sets the packet size (flits) */
//						slot.pkt_size = 3 * 2;

						SendCtrlPacket(proc, TASK_ALLOCATED, source_processor, requesting_task, 0);

//						DMA_Send(&slot);

						/* Reads incoming NoC services while message is transmited */
						ReadIncomingServices();
					}
					
					//allocates the task if there is an available processor
					else { 
					
					if(task_terminated[taskID] != TERMINATED) {
						if ((taskList[taskID].type==PLASMA && free_plasmas>0) || (taskList[taskID].type==MBLITE && free_mblites>0) || (task_pes[taskID]!= -1))
						{
							/* Searches the task code in the tasks repository */
							for(i = 0; i < total_tasks; i++)
							{
								repository_taskID = task[i].id & 0x7fffffff; //logic to unset the task processor type;
							
								if(taskID == repository_taskID)
									break;
									
							}

							/* Gets an available processor */
							premapped = 0;
							task_type = taskList[taskID].type;
							if(task_pes[taskID] == -1)
							{
								proc = SearchVS(taskID, task_type);
								//if(task_pes[taskID]!= -1) premapped = 1;
							}
							else
							{
								proc = task_pes[taskID];
								premapped = 1;
							}
							
							if ( proc == NONE) {
								puts("# ERROR: proc: "); puts(itoa(proc)); puts(" invalido.\n");
								for(;;);
							}

							/* Allocates the task */
//							TaskAllocation(proc, &task[i]);
							task_pes[task->id & 0x7fffffff] = proc;
							SendCtrlPacket(proc, TASK_ALLOCATION, 0, 0, &task[i]);
							
							/* Reads incoming NoC services while the task is transmited */
							ReadIncomingServices();

							/* Removes the allocated task from the task_request array */
							if ( !incoming_service ) {	/* pendent_task_request */
								for(j=0; j<MAX_GLOBAL_TASKS*2; j++)
									if ( task_request[j].requested_task == taskID && task_request[j].source_processor == source_processor && task_request[j].requesting_task == requesting_task) {
										task_request[j].requested_task = NONE;
										break;
									}
							}

							InsertTaskLoc(taskID, proc);	/* Updates the task_location table */
							
							if(!premapped)
							{
								PageUsed(proc); /* Updates the 'proc' free pages */
							}
							if(pe_free_pages[proc >> 4][proc & 0xF]==MAX_LOCAL_TASKS-1 && MAX_LOCAL_TASKS>1) premapping(proc, taskID);
							
							/*** Sends the requested task location to the requesting task ***/

							/* Sets the requesting task processor */
//							slot.remote_addr = source_processor;

							/* Sets the service */
//							slot.service = TASK_ALLOCATED;

							/* Sets the processor where the requested task has been allocated */
//							slot.arg[0] = proc;

							/* Sets the requested task */
//							slot.arg[1] = task[i].id;

							/* Sets the packet size (flits) */
//							slot.pkt_size = 3 * 2;

							SendCtrlPacket(source_processor, TASK_ALLOCATED, proc, task[i].id, 0);

//							DMA_Send(&slot);

							/* Reads incoming NoC services while the message is transmited */
							ReadIncomingServices();


							/*** Sends the requesting task location to the requested task ***/

							/* Sets the requested task processor */
//							slot.remote_addr = proc;

							/* Sets the service */
//							slot.service = TASK_ALLOCATED;

							/* Sets the processor where the requesting task is allocated */
//							slot.arg[0] = source_processor;

							/* Sets the requesting task */
//							slot.arg[1] = requesting_task;

							/* Sets the packet size (flits) */
//							slot.pkt_size = 3 * 2;

							SendCtrlPacket(proc, TASK_ALLOCATED, source_processor, requesting_task, 0);

//							DMA_Send(&slot);

							/* Reads incoming NoC services while the message is transmited */
							ReadIncomingServices();

						}
						//there is no available processor, so store the requested task to be mapped later
						else 
						{
							if ( incoming_service ) {	/* Stores the task request */

								/* Stores a TASK_REQUEST service */
								for(i=0; i<MAX_GLOBAL_TASKS*2; i++)
									if ( task_request[i].requested_task == NONE ) {
										task_request[i].requested_task = taskID;
										task_request[i].requesting_task = requesting_task;
										task_request[i].source_processor = source_processor;
										break;
									}

								/* There is no space to store the task request */
								if ( i == MAX_GLOBAL_TASKS*2 ) {
									puts("# There is no space to store the task request.\n");
									puts("Kernel panic!!!\n");
									for(;;);
								}
							}
						}
					}
					else{
							for(j=0; j<MAX_GLOBAL_TASKS*2; j++)
								if ( task_request[j].requested_task == taskID && task_request[j].source_processor == source_processor && task_request[j].requesting_task == requesting_task) {
									task_request[j].requested_task = NONE;
									break;
								}
					}
				}
        		break;

		case TASK_TERMINATED:

			/* Verifies if the TASK_TERMINATED is incoming (NoC) or stored (task_terminated[]) */
			if ( incoming_service ) {

			    	/* Reads the processor where the task has been allocated */
				NI_Read();

				/* Reads the terminated task */
				taskID = NI_Read();
			}
					
			task_location[taskID].task = NOT_ALLOCATED;
					
                    	there_is_mapped_tasks = 0;
                    
                   	for (i=0; i<MAX_GLOBAL_TASKS; i++){
                        	if (task_location[i].task != NOT_ALLOCATED){
                    			there_is_mapped_tasks = 1;
		break;
                        }
                    }
                    

					proc = task_location[taskID].processor;	/* Gets the terminated task location */

					PageReleased(proc);		/* Updates the 'proc' free pages */
					
					task_terminated[taskID] = TERMINATED;
					
                    if (there_is_mapped_tasks == 0){
                        MemoryWrite(END_SIM,1);
                    }
                    
                    
                    for (i=0; i<MAX_GLOBAL_TASKS*2; i++){
						if ( task_request[i].requested_task == taskID ) {
							task_request[i].requested_task = NONE;
						}
						if ( location_request[i].requested_task == taskID ) {
							location_request[i].requested_task = NONE;
						}
					}



        		break;

				case DEBUG_MESSAGE:
				    DebugMessage();	/* Handles an Echo() */
				break;

				case LOCATION_REQUEST:

					/* Verifies if the LOCATION_REQUEST is incoming (NoC) or stored (location_request[]) */
					if ( incoming_service ) {

						/* Reads the source processor */
						source_processor = NI_Read();

						/* Reads the requesting task */
						requesting_task = NI_Read();

						/* Reads the requested task */
						taskID = NI_Read();
					}

					/* Searches the task location  */
					if ( task_location[taskID].task == taskID ) {
							
							/* Sets the requesting processosr */
//							slot.remote_addr = source_processor;

							/* Sets the service */
//							slot.service = LOCATION_ANSWER;

							/* Sets the requesting task */
//							slot.arg[0] = requesting_task;

							/* Sest the requested task */
//							slot.arg[1] = taskID;
							
							/* Sets the processor where the requested task is allocated */
//							slot.arg[2] = task_location[taskID].processor;
							
							/* Sets the packet size */
//							slot.pkt_size = 4 * 2;

							SendCtrlPacket(source_processor, LOCATION_ANSWER, taskID, task_location[taskID].processor, 0);

//							DMA_Send(&slot);

							/* Reads incoming NoC services while the message is transmited */
							ReadIncomingServices();

							if ( !incoming_service ) {	/* pendent_task_location */

								/* Removes the location requested from the location_request array */
								for (j=0; j<MAX_GLOBAL_TASKS*2; j++)
									if ( location_request[j].requested_task == taskID && location_request[j].source_processor == source_processor && location_request[j].requesting_task == requesting_task ) {
										location_request[j].requested_task = NONE;
										break;
									}
							}
					}

					if ( task_location[taskID].task != taskID && incoming_service ) {	/* Task not allocated yet */

						/* Stores a LOCATION_REQUEST service */
						for(i=0; i<MAX_GLOBAL_TASKS*2; i++) {

							/* Stores the the location request */
							if ( location_request[i].requested_task == NONE ) {
								location_request[i].requested_task = taskID;
								location_request[i].requesting_task = requesting_task;
								location_request[i].source_processor = source_processor;
								break;
							}
						}

						/* There is no space to store the location request */
						if ( i == MAX_GLOBAL_TASKS*2) {
							puts("# There is no space to store the location request.\n");
							puts("# Kernel panic!!!\n");
							for(;;);
						}
					}
				break;
				
				case NEW_TASK:
					
					if(taskID>=total_tasks_old)
					{
						for(i=total_tasks_old; i<total_tasks; i++) 
						{
							if(task[i].id >= 0x80000000)
							{
								taskList[i].type = MBLITE;
							}
							else
							{
								taskList[i].type = PLASMA;
							}
							
							cont = 0;
							for(j = 0; j<10; j++)
							{
								if(task[i].dependences[j].task != DYNAMIC)
								{
									taskList[i].taskDep[j].task = task[i].dependences[j].task;
									taskList[i].taskDep[j].flits = task[i].dependences[j].flits;
									cont++;
								}
							}
							taskList[i].dependences = cont;
						}
					}
					
					/* Searches the requested task location */
					if (task_location[taskID].task == taskID) //break;
					{
						puts("#Task ");
						puts(itoa(taskID));
						puts(" is already allocated!\n");
						MemoryWrite(ACK_TASK,1);
					} 
					else 
					{
						if(task[taskID].id >= 0x80000000)
						{
							task_type=MBLITE;
						}
						else
						{
							task_type=PLASMA;
						}
						
						/* Gets an available processor */
						proc = InitialMapping(task_type);
						
						if ( proc!=NONE )
						{
							
							puts("#New external task ");
							puts(itoa(task[taskID].id & 0x7fffffff));
							puts("!\n");
							puts(itoa(taskID));						
							puts("!\n");
							puts(" allocated at ");
							puts(itoa(proc));
							puts("!\n");
							/* Allocates the task */
//							TaskAllocation(proc, &task[taskID] );
							task_pes[task->id & 0x7fffffff] = proc;
							SendCtrlPacket(proc, TASK_ALLOCATION, 0, 0, &task[taskID]);
							/* Reads incoming NoC services while the task is transmited */
							ReadIncomingServices();
							PageUsed(proc);    /* Updates the 'proc' free pages */
							InsertTaskLoc(taskID, proc);	/* Updates the task_location table */
							if(pe_free_pages[proc >> 4][proc & 0xF]==MAX_LOCAL_TASKS-1 && MAX_LOCAL_TASKS>1) premapping(proc, taskID);
							MemoryWrite(ACK_TASK,1);
						}
						else
						{
							puts("# ERROR: proc: "); puts(itoa(proc)); puts(" invalido.\n");
							for(;;);
						}
					}
				break;

				default:
		    		puts("# DEFAULT desconhecido, ");
				   	puts("service: ");
				   	puts(itoh(service));
				   	puts("\n");

					for(;;);
        		break;
			}
		}

		pendent_task_terminated = FALSE;
		pendent_task_request = FALSE;
		pendent_location_request = FALSE;


		/* Searches a pendent TASK_TERMINATED */
		for (i=0; i<MAX_GLOBAL_TASKS; i++)
			if ( task_terminated[i] != NONE && task_terminated[i] != TERMINATED ) {
				taskID = task_terminated[i];
				pendent_task_terminated = TRUE;
				break;
			}
		
		if ( pendent_task_terminated )
			continue;


		/* Searches a pendent TASK_REQUEST, if there is no pendent TASK_TERMINATED */
		for (i=0; i<MAX_GLOBAL_TASKS*2; i++)
			if ( task_request[i].requested_task != NONE ) {
				taskID = task_request[i].requested_task;
				requesting_task = task_request[i].requesting_task;
				source_processor = task_request[i].source_processor;
				pendent_task_request = TRUE;
				break;
			}

		if ( pendent_task_request )
			continue;
			
		/* Searches a pendent LOCATION_REQUEST, if there is no pendent TASK_TERMINATED or TASK_REQUEST */
		for (i=0; i<MAX_GLOBAL_TASKS*2; i++)
			if ( location_request[i].requested_task != NONE ) {
				taskID = location_request[i].requested_task;
				requesting_task = location_request[i].requesting_task;
				source_processor = location_request[i].source_processor;
				pendent_location_request = TRUE;
				break;
		}
	}

    return 0;
}
