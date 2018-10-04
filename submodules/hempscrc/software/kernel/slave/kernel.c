/*--------------------------------------------------------------------------------
* TITLE: MBLite and Plasma V3 microkernel
* AUTHOR: CARLO LUCAS
* DATE CREATED: 11/04/2011
* FILENAME: kernelV3.c
* PROJECT: Hemps-S
* DESCRIPTION: Microkernel for Plasma and MBLite processors
* 			   Based in the microkernel kernel_V2_MBlite.c for the MBLite processor
* 			   and in the microkernel kernel.c (V2_Plasma) for the Plasma processor
*
--------------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------------
 * INCLUDE
--------------------------------------------------------------------------------*/

#include "kernel.h"
#include "../../include/stdlib.h"
#include "../../include/services.h"
#include "../../include/communication.h"
#include "../../include/seek.h"

// #define SERIALCOMM
#define SEEK_TIMES
/*-------------------------------------------------------------------------------
 * GLOBALS
 * -----------------------------------------------------------------------------*/
 
#ifdef PLASMA
unsigned int* gp_ptr;               /*kernel global pointer*/
#endif
unsigned int* sp_ptr;              /*kernel stack pointer*/			

unsigned int bug_syscall;			/*indicates when scheduling is needed*/


PipeSlot *last_pipe_slot_sent;		/* Points the pipe slot which is being transmited by DMA */ 
PipeSlot slot1, slot2;				/* Temporary slots */	
RequestMessage  requestMessage[REQUEST_SIZE]; /*messages requests table	*/

TCB idle_tcb;						/*idle task's tcb*/
TCB tcbs[MAXLOCALTASKS];			/*local tasks' tcbs*/

TCB *current;						/*pointer to scheduled task's tcb*/
TCB *allocatingTCB;					/*pointer to tcb which is being allocated*/

//int *str_part;                      /*used by puts*/

tableSlot table;
// tableSlot table_SW[14];
unsigned int target_seek;

unsigned int last_sqnc_nbr[MAX_GLOBAL_TASKS];

unsigned int seek_lock;

lost_ackSlot lost_acks[LOST_ACKS_SIZE];

msg_requestSlot msg_request[MSG_REQUEST_SIZE];

unsigned int last_print;

 
/*-------------------------------------------------------------------------------
 * FUNCTIONS
 *------------------------------------------------------------------------------*/
 

/*-------------------------------------------------------------------------------
* GetFreeSlot
*
* DESCRIPTION:
*	Returns a pointer to one of the temporary slots (slot1 or slot2).
*	While a temporary slot is accessed by DMA (USED), the other one is available (EMPTY).
*
*--------------------------------------------------------------------------------*/
PipeSlot* GetFreeSlot() {

	if ( slot1.status == EMPTY ) {
		slot1.status = USED;
		slot2.status = EMPTY;
		return &slot1;
	}
	else {
		slot2.status = USED;
		slot1.status = EMPTY;
		return &slot2;
	}
}

/* Searches for TERMINATED tasks */
void SearchTerminatedTasks(){
    int i,j;
    //int task_waiting_ack;
    int nbr_task_waiting_ack;
    //PipeSlot *slot_ptr;
    
    nbr_task_waiting_ack=1;
    for(i=0; i<MAXLOCALTASKS; i++)
        if ( tcbs[i].status == TERMINATED ) {

            for(j=0; j<PIPE_SIZE; j++){
                if (pipe[j].source == tcbs[i].id){
                    if(pipe[j].status == WAITING_ACK){
                        // task_waiting_ack = j;
                        pipe[j].status = EMPTY;
                        puts("W");
                    }
                    else if(pipe[j].status == USED){
                        puts("U");
                        nbr_task_waiting_ack = 0;
                        break;
                    }
                    else if(pipe[j].status == EMPTY){
                        puts("E");
                    }
                }
            }
            puts("\n");

            //if there is only one message waiting ack AND there is no other message from this id
            //in the pipe, it means that it is the last message, and we assume that this message
            //has been delivered: we can set the status as EMPTY
            // if(nbr_task_waiting_ack == 1){
            //     pipe[task_waiting_ack].status = EMPTY;
            //     puts("last message!\n");
            // }


            /* searches for  messages from this task in pipe*/
            for(j=0; j<PIPE_SIZE; j++)
                if (pipe[j].source == tcbs[i].id && pipe[j].status != EMPTY)
                    break;

            if (j == PIPE_SIZE) {	/* no messages in pipe*/
                tcbs[i].status = FREE;

                task_location[tcbs[i].id].task = NOT_ALLOCATED;
                request_task[tcbs[i].id] = -1;
                location_request_task[tcbs[i].id] = -1;

                puts("TASK_TERMINATED\t");
                puts(itoa(tcbs[i].id));
                puts("\t");	
                puts(itoa(MemoryRead(TICK_COUNTER)));
                puts("\n");

                SendCtrlPacket(MASTERADDRESS, TASK_TERMINATED, tcbs[i].id, 0, 0);
                
                for(j=0; j<MAX_GLOBAL_TASKS; j++)
                {
                    if(task_location[j].task != NOT_ALLOCATED && task_location[j].processor!=net_address)
                    {
                        //Send TASK_DEALLOCATED
                        SendCtrlPacket(task_location[j].processor, TASK_DEALLOCATED, tcbs[i].id, 0, 0);
                    }
                }
            }
        }
}

/*--------------------------------------------------------------------
* InsertMessageRequest - DONE!
*
* DESCRIPTION:
*    Inserts a new message request in the 'requestMessage' array.
*
*--------------------------------------------------------------------*/
int InsertMessageRequest(int target, int source) {

    int i;

    for (i=0; i<REQUEST_SIZE; i++)
    	if ( requestMessage[i].requesting == -1 ) {
			requestMessage[i].requesting  = target;
    		requestMessage[i].requested  = source;
    		return TRUE;
		}

	return FALSE;	/*no space in table*/
}

/*--------------------------------------------------------------------
* GetRequestingMessage - DONE!
*
* DESCRIPTION:
*    Removes a stored message request.
*
*--------------------------------------------------------------------*/
int GetRequestingMessage(int source, int target) {

    int i, requesting;

    for(i=0; i<REQUEST_SIZE; i++) {
        if( requestMessage[i].requested == source && requestMessage[i].requesting == target){
            requesting = requestMessage[i].requesting;
            requestMessage[i].requesting = -1;   /* get out the request*/
            return requesting;
        }
    }

    return -1;
}

/*--------------------------------------------------------------------
* SearchRequestingMessage - DONE!
*
* DESCRIPTION:
*    Verifies if there is a stored message resquest from 'target'.
*
*--------------------------------------------------------------------*/
int SearchRequestingMessage(int target) {

    int i;

    for(i=0; i<REQUEST_SIZE; i++)
        if(  requestMessage[i].requesting == target )
            return requestMessage[i].requested;

    return -1;
}

/*--------------------------------------------------------------------
* SearchTCB
*
* DESCRIPTION:
*    Returns a pointer to TCB. ONLY for local tasks.
*--------------------------------------------------------------------*/
TCB* SearchTCB(unsigned int task) {

    int i;

    for(i=0; i<MAXLOCALTASKS; i++)
    	if(tcbs[i].id == task)
    		return &tcbs[i];

    return 0;
}


/*--------------------------------------------------------------------------------------------------------
* GetSlotFromPipe - DONE!
*
* DESCRIPTION:
*    Returns a pointer to the requested message (NULL if it is not available).
*--------------------------------------------------------------------------------------------------------*/
PipeSlot* GetSlotFromPipe(int target, int source, enum PipeSlotStatus status) {

    PipeSlot* value;
    char str[50];
    
    int earliest = 0, position=0;
    int i = 0;
    int found = 0;

    /*searches for the requested message*/
       for(i=0; i<PIPE_SIZE; i++) {
    
        if (pipe[i].status == status && pipe[i].source == source ) {
            if (pipe[i].target == target) {
                found = 1;

                 /*determines where is the earliest message for this target*/
                if (earliest == 0 || pipe[i].order < earliest) {
                    earliest = pipe[i].order;
                    position = i;
                }
            }
        }
    }
   if(!found) /*	no message for this target*/
        value = (PipeSlot*)0;
    else
        value = &(pipe[position]);


    return value;
}

unsigned int add_msg_request(unsigned int processor, unsigned int target_id, unsigned int source_id){
    int i;
    
    for(i=0;i<MSG_REQUEST_SIZE;i++){
        if(msg_request[i].status == EMPTY){
            msg_request[i].status = USED;
            msg_request[i].processor = processor;
            msg_request[i].target_id = target_id;
            msg_request[i].source_id = source_id;
            //set the time which i've sent the msg_request
            msg_request[i].last_msg_request_time = MemoryRead(TICK_COUNTER);
            return 1;
        }
    }
    return 0;
}

unsigned int remove_msg_request(unsigned int processor, unsigned int target_id, unsigned int source_id){
    int i;
    unsigned int time;
    char str [50];
    unsigned int time_between_req_deliver;
  
    for(i=0;i<MSG_REQUEST_SIZE;i++){
        if(msg_request[i].status    == USED &&
           msg_request[i].processor == processor &&
           msg_request[i].source_id == source_id &&
           msg_request[i].target_id == target_id){
           msg_request[i].status = EMPTY;
                  
            return 1;
        }
    }
    return 0;
}

unsigned int resend_msg_request(unsigned int processor){
    int i;
    int value = 0;
    TCB *target;
    PipeSlot *slot_ptr; 
      
    for(i=0;i<MSG_REQUEST_SIZE;i++){
        if(msg_request[i].status == USED && msg_request[i].processor == processor){
            slot_ptr = GetFreeSlot();
            SendCommPacket(msg_request[i].processor, MESSAGE_REQUEST, msg_request[i].target_id, msg_request[i].source_id, 0, slot_ptr);
            msg_request[i].last_msg_request_time = MemoryRead(TICK_COUNTER);
            value = 1;
        }
    }
    return value;
}

unsigned int resend_msg_request_xy(unsigned int processor){
    int i;
    int value = 0;
    TCB *target;
    PipeSlot *slot_ptr; 

    for(i=0;i<MSG_REQUEST_SIZE;i++){
        if(msg_request[i].status == USED && msg_request[i].processor == processor){
            slot_ptr = GetFreeSlot();
            SendCommPacket(msg_request[i].processor, MESSAGE_REQUEST, msg_request[i].target_id, msg_request[i].source_id, 0, slot_ptr);
            msg_request[i].last_msg_request_time = MemoryRead(TICK_COUNTER);
            value = 1;
        }
    }
    return value;
   
}

//finds a free position in the pipe, the argument max_order, returns the maximum sequence number of the source-target
int find_free_pipe_position(unsigned int target_id, int *max_order){
    char str[50];
    int i;
    int free_position = -1;
    *max_order = 0;
    
    for(i=0; i<PIPE_SIZE; i++) { /*searches for a free position*/
       
        if (pipe[i].order > *max_order)
            *max_order = pipe[i].order;

        if (pipe[i].status == EMPTY)
            free_position = i;
    }
    
    return free_position;
}

/*--------------------------------------------------------------------
* Syscall
*
* DESCRIPTION:
*    Syscall handler.
*--------------------------------------------------------------------*/
int Syscall(unsigned int service, unsigned int arg0, unsigned int arg1, unsigned int arg2) {

    int i, j, length;
    int reader_id = -1;
    int reader_processor = -1;
    int task_processor;
    int value;
    int pos = -1;
    int nbr_task_waiting_ack;
    int task_waiting_ack;

    int max_order;
    int free_position;

    char *ptr, *ptr_msg;
    int pkt_size;

    unsigned int source_id;
    unsigned int target_id;
    
    Message *msg_read;
    Message *msg_write;
    PipeSlot *slot_ptr;

    TCB *requestingTCB;

    bug_syscall = FALSE;
    value = FALSE;
    
    char str[50];

    switch(service) {
		case EXIT:
                                
            puts("EXIT\n");
          
			current->status = TERMINATED;

			/* searches for  messages from this task in pipe*/
            nbr_task_waiting_ack = 1;
            for(j=0; j<PIPE_SIZE; j++){
                if (pipe[j].source == current->id){
                    if(pipe[j].status == WAITING_ACK){
                        pipe[j].status = EMPTY;
                        puts("W");
                    }
                    else if(pipe[j].status == USED){
                        puts("U");
                        nbr_task_waiting_ack = 0;
                        break;
                    }
                    else if(pipe[j].status == EMPTY){
                        puts("E");
                    }
                }
            }
            puts("\n");

            //if there is only one message waiting ack AND there is no other message from this id
            //in the pipe, it means that it is the last message, and we assume that this message
            //has been delivered: we can set the status as EMPTY
            if(nbr_task_waiting_ack != 0){
                /* no messages in pipe*/
				current->status = FREE;
				
                puts("\nTASK_TERMINATED\t");
                puts(itoa(current->id));
                puts("\t");
                puts(itoa(MemoryRead(TICK_COUNTER)));
                puts("\n");
                
				/* Sets the task as not allocated */
				task_location[current->id].task = NOT_ALLOCATED;
				request_task[current->id] = -1;
				location_request_task[current->id] = -1;

                //sends a task terminated packet to the master
                SendCtrlPacket(MASTERADDRESS, TASK_TERMINATED, current->id, 0, 0);

			    for(j=0; j<MAX_GLOBAL_TASKS; j++)
			    {
					if(task_location[j].task != NOT_ALLOCATED && task_location[j].processor!=net_address)
					{
                        //Send TASK_DEALLOCATED
                        SendCtrlPacket(task_location[j].processor, TASK_DEALLOCATED, current->id, 0, 0);
					}
			    }
		
			}

			bug_syscall = TRUE;

			/* return true;*/
			value = TRUE;
		break;

		case WRITEPIPE:
             
			/* Avoids blocking in a system call, if DMA is not available (deadlock prevention) */
			if ( MemoryRead(DMA_ACTIVE) ){
				return FALSE;
			}
			
			target_id = (unsigned int) arg1;
			source_id = current->id;

			/*Points the message in the task page. Address composition: offset + msg address*/
			msg_read = (Message *)((current->offset) | arg0);


			/*** Verifies if the target task is already allocated ***/

			/* Searches the target task location */
			reader_processor = -1;
			reader_id = -1;
			if ( task_location[target_id].task == target_id ) {
				reader_processor = task_location[target_id].processor;

				/* Searches for a request for this message */
				reader_id = GetRequestingMessage(source_id, target_id);
			}
	
			if (reader_processor == -1) {	/* Target task not allocated yet */

				if ( request_task[target_id] != target_id ){
							
					request_task[target_id] = target_id;
					location_request_task[target_id] = target_id;

                    //sends the TASK_REQUEST packet
                    SendCtrlPacket(MASTERADDRESS, TASK_REQUEST, target_id, source_id, 0);
				}
			}


			/*** If there is a request to the message and the target task location is knew send it, else stores it in the pipe ***/


			if(reader_id != -1 && reader_processor != -1 ) {   /* There is a request for this message and the target task location is knew */

				if (reader_processor == net_address) {	/* Local request */
					/*requesting is local*/
					requestingTCB = SearchTCB(reader_id);
					msg_write = (Message*)((requestingTCB->offset) | ((unsigned int)requestingTCB->reg[3])); /*reg[3] = address message*/

					/*stores the message length*/
					length = msg_write->length = msg_read->length;

					/*stores the entire message*/
					for (i=0; i<length; i++)
						msg_write->msg[i] = msg_read->msg[i];

					requestingTCB->reg[0] = TRUE;  /*v0, return value*/

					requestingTCB->status = READY;
				}
				else {	/* Remote request */

                    /** MODIFIED FOR ACK_MESSAGE_DELIVERY **/
                    free_position = find_free_pipe_position(target_id, &max_order);
                    
                    max_order = max_order + 1;

                    if(free_position == -1) { /*full pipe*/
                        bug_syscall = TRUE;	/* Schedules a new task if any */
                        value = FALSE;	/* return false*/
                    }
                    else {	/* Stores the message in the pipe */

                        /* Pointer used to increase the performance on the pipe access */
                        slot_ptr = &pipe[free_position];

                        /* Verifies if 'slot_ptr' points the pipe slot which is being transmited by DMA */
                        if (slot_ptr == last_pipe_slot_sent) {
                            last_pipe_slot_sent = 0;

                            /* Waits the DMA finish the transmition before set the new slot parameters */
                            while ( MemoryRead(DMA_ACTIVE) );
                        }
                        /*stores the message's length*/
                        slot_ptr->length = length = msg_read->length;

                        /*stores the message's target*/
                        slot_ptr->target = target_id;

                        /*stores the message's source*/
                        slot_ptr->source = source_id;

                        /*sets the pipe slot status*/
                        slot_ptr->status = WAITING_ACK;
                        
                        /*sets the time i've sent the packet*/
                        slot_ptr->last_msg_delivery_time = MemoryRead(TICK_COUNTER);

                        /*stores the message order*/
                        slot_ptr->order = max_order;
                                                
                        /*stores the entire message*/
                        for (i = 0; i < length; i++)
                            slot_ptr->message[i] = msg_read->msg[i];
                        
                    }
                    /** MODIFIED FOR ACK_MESSAGE_DELIVERY **/
                    
                    //set the time i've sent the message
                    slot_ptr->last_msg_delivery_time = MemoryRead(TICK_COUNTER);

                    //Sends the message
                    #ifdef SERIALCOMM
                        if (there_is_waiting_ack){//there is a message with status WAITING_ACK, wait for send this message
                            slot_ptr->status = TO_BE_SENT;
                            slot_ptr->remote_addr = reader_processor;
                        }
                        else{
                            // puts("msg req\n");
                            SendCommPacket(reader_processor, MESSAGE_DELIVERY, 0, 0, 0, slot_ptr);
                        }
                    #else
                        SendCommPacket(reader_processor, MESSAGE_DELIVERY, 0, 0, 0, slot_ptr);
                    #endif
				}

				/* return true*/
				value = TRUE;
			}
			else {	/* There is not a request for this message or there is a request for this message but the target location is unknow yet */
					/* Stores the message in the pipe */

                free_position = find_free_pipe_position(target_id, &max_order);
                
				max_order = max_order + 1;

				if(free_position == -1) { /*full pipe*/
					bug_syscall = TRUE;	/* Schedules a new task if any */
					value = FALSE;	/* return false*/
				}
				else {	/* Stores the message in the pipe */

					/* Pointer used to increase the performance on the pipe access */
					slot_ptr = &pipe[free_position];

					/* Verifies if 'slot_ptr' points the pipe slot which is being transmited by DMA */
					if (slot_ptr == last_pipe_slot_sent) {
						last_pipe_slot_sent = 0;

						/* Waits the DMA finish the transmition before set the new slot parameters */
						while ( MemoryRead(DMA_ACTIVE) );
					}
					/*stores the message's length*/
					slot_ptr->length = length = msg_read->length;

					/*stores the message's target*/
					slot_ptr->target = target_id;

					/*stores the message's source*/
					slot_ptr->source = source_id;

					/*sets the pipe slot status*/
					slot_ptr->status = USED;

					/*stores the message order*/
					slot_ptr->order = max_order;

					/*stores the entire message*/
					for (i = 0; i < length; i++)
						slot_ptr->message[i] = msg_read->msg[i];

					/*return true*/
					value = TRUE;
				}
			}


		break;

		case READPIPE:

            //puts("READPIPE\n");
			/* Avoids blocking in a system call, if DMA is not available (deadlock prevention) */
			if ( MemoryRead(DMA_ACTIVE) )
				return FALSE;

			source_id = (unsigned int) arg1;
			target_id = current->id;

					
			/*searches for the message source location*/
			task_processor = -1;
			if ( task_location[source_id].task == source_id ) {
				task_processor = task_location[source_id].processor;
			}

			if (task_processor == net_address) {	/* Local read */
				
				slot_ptr = GetSlotFromPipe(target_id, source_id, USED);

				if (slot_ptr) {
					
					msg_write = (Message*) arg0;

					/*address composition: offset + msg address*/
					msg_write = (Message*)((current->offset) | ((unsigned int)msg_write));

					/*copies the message length*/
					length = msg_write->length = slot_ptr->length;

					/*copies the entire message*/
					for (j = 0; j<length; j++)
						msg_write->msg[j] = slot_ptr->message[j];

					slot_ptr->status = EMPTY;
					
					/* Sets the task as not allocated */
					for(j=0; j<MAXLOCALTASKS; j++)
						if ( tcbs[j].id == source_id && tcbs[j].status != TERMINATED ) {
							break;
						}

					if (j == MAXLOCALTASKS) {	/* no messages in pipe*/

						for(i=0; i<PIPE_SIZE; i++)
							if(pipe[i].status != EMPTY && pipe[i].source == source_id)
								break;

						if (i == PIPE_SIZE) {	/* no messages in pipe*/
							requestingTCB = SearchTCB(source_id);

							requestingTCB->status = FREE;
							puts("TASK_TERMINATED\t");
							puts(itoa(source_id));
							puts("\t");
							puts(itoa(MemoryRead(TICK_COUNTER)));
							puts("\n");

                            //Sends the packet
                            SendCtrlPacket(MASTERADDRESS, TASK_TERMINATED, source_id, 0, 0);
							
							puts("TASK_DEALLOCATED\t");
							puts(itoa(source_id));
							puts("\t");
							puts(itoa(MemoryRead(TICK_COUNTER)));
							puts("\n");
							
							for(j=0; j<MAX_GLOBAL_TASKS; j++)
							{
								if(task_location[j].task != NOT_ALLOCATED && task_location[j].processor!=net_address)
								{
									/* Sets the target processor, switching mode and packet priority */
									puts("to proc ");
									puts(itoa(task_location[j].processor));
									puts("\n");

                                    //sends the packet TASK_DEALLOCATED
                                    SendCtrlPacket(task_location[j].processor, TASK_DEALLOCATED, source_id, 0, 0);
								}
							}
						}
					}	

					value = TRUE;
				}
				else {
					/*no message for this target*/
					/*inserts a new entry in the requestMessage table*/
					InsertMessageRequest(target_id, source_id); /* RED WARNING: Valor de retorno não testado */

					/*sets status to waiting*/
					current->status = WAITING;
					bug_syscall = TRUE;

					/* return false*/
					value = FALSE;
				}
			}
			else if (task_processor == -1) {	/* Message source not allocated yet */

				if ( location_request_task[source_id] == -1) {	/* Request the task */

					location_request_task[source_id] = source_id;

					    //sends the LOCATION_REQUEST
					    SendCtrlPacket(MASTERADDRESS, LOCATION_REQUEST, target_id, source_id, 0);
				}

				bug_syscall = TRUE;

				/* return false*/
				value = FALSE;
			}
			else {	/* task is allocated, but is remote*/
					
				/* Gets a temporary slot */
				slot_ptr = GetFreeSlot();

				//sends the MESSAGE_REQUEST
				SendCommPacket(task_processor, MESSAGE_REQUEST, target_id, source_id, 0, slot_ptr);
                
                if(!add_msg_request(task_processor, target_id, source_id)){
                    puts("something went wrong with add_msg_request\n");
                }
                
				current->status = WAITING;
				bug_syscall = TRUE;

				/* return false*/
				value = FALSE;
			}

		break;

		case GETTICK:
			value = MemoryRead(TICK_COUNTER);

		break;

		case ECHO:
			
            //puts("ECHO\n");
			/* Avoids blocking in a system call, if DMA is not available (deadlock prevention) */
			//while( MemoryRead(DMA_ACTIVE) );
			//if( MemoryRead(DMA_ACTIVE) )
				//return FALSE;

			///* Gets a temporary slot */
			//slot_ptr = GetFreeSlot();

			///* Points the string at the task memory page */
			//ptr = (char *)((current->offset) | (unsigned int) arg0);

			///* Cast the message field (unsigned int message)*/
			//ptr_msg = (char *)slot_ptr->message;

			//i = 0;

			///* Copies the string from task memory page to kernel */
			//while(*ptr)
				//ptr_msg[i++] = *ptr++;

			///* Sets the string end */
			//if (i%4 > 0)
				//ptr_msg[i] = 0;

			///* Sets the NoC packet size (flits): string length + header  */
			//pkt_size = i >> 1;

			///* String chars packet in 4 bytes integers */
			//if ( (i & 3) == 1 )
				//pkt_size += 2;

			//else if ( (i & 3) == 2 || (i & 3) == 3 )
				//pkt_size += 1;

			//pkt_size += 5 * 2;		/* Adds the header size (service, local_addr, target, source, length)*/

			///* Sets the remote processor address */
			////slot_ptr->remote_addr = MASTERADDRESS;

			/////* Sets the NoC packet size (flits) */
			////slot_ptr->pkt_size = pkt_size;
            
			/////* Sets the packet service */
			////slot_ptr->service = DEBUG_MESSAGE;
            
			/////* Sets the source task */
			////slot_ptr->source = current->id;

			/////* Sets the string length (bytes) */
			////slot_ptr->length = i;
			
            ////DMA_Send(slot_ptr);
            
            ///* Sets the remote processor address */
            //NI_Write(MASTERADDRESS);
            ///* Sets the NoC packet size (flits) */
            //NI_Write(pkt_size);
            ///* Sets the packet service */
            //NI_Write(DEBUG_MESSAGE);
            ///* Sets the source processor */
            //NI_Write(net_address);
            ///* Sets the target task */
            //NI_Write(current->id);//this field is NOT used!
            ///* Sets the source task */
            //NI_Write(current->id);
            ///* Sets the string length (bytes) */
            //NI_Write(i);
            
            //if(i%4==0){
                //i=i/4;
            //}
            //else{
                //i=i/4+1;
            //}
            
            //for(j=0;j<i;j++){
                //NI_Write(slot_ptr->message[j]);
            //}
            
			/* return true*/
			value = TRUE;
		break;
        
		default:
			/*return false*/
			value = FALSE;
		break;
    }

    return value;
}

void seek_path(unsigned int remote_addr, int reason){
    char str[50];
    
    seek_lock = 1;
    target_seek = remote_addr;
    MemoryWrite(SEEK_REG,remote_addr);
}

void print_port(unsigned int port){
    switch(port){
        case 0x0: case 0x4: puts("E"); break;
        case 0x1: case 0x5: puts("W"); break;
        case 0x2: case 0x6: puts("N"); break;
        case 0x3: case 0x7: puts("S"); break;
        default: puts("-"); break;
    }
}

void SeekUnreachable(){
    unsigned int target, service;
    char str[50];
    MemoryWrite(CLEAR_REG,target);//clear
    target = MemoryRead(SEEK_REG);
    service = MemoryRead(SEEK_SERVICE_REG);
      puts(":unreacheable\n");
     
    #ifdef SEEK_TIMES
   
    #endif

    seek_path(target, 555);
}

void SeekResend(){
    

    unsigned int target, service;
    char str[50];
    MemoryWrite(CLEAR_REG,target);//clear
    target = MemoryRead(SEEK_REG);
    service = MemoryRead(SEEK_SERVICE_REG);
    puts(":seek resend\n");
    #ifdef SEEK_TIMES
    #endif
    
	resend_messages_xy(target);
	resend_msg_request_xy(target);
	
}

void receive_seek(unsigned int first_flit){
    int i,j;
    unsigned int flit;
    unsigned int flit_shifted;
    unsigned int flit_masked;
    unsigned int flit_shifted_masked;
    unsigned char algorithm;
    unsigned int shift;
    unsigned int inicio;
   
    inicio = MemoryRead(TICK_COUNTER);
    
    //shifts 16 times only the first flit;
    flit = (first_flit&0x0000ffff) << 16;
    flit_shifted = flit << 2;
    
    j=0;//variable j is used to index the final path
    i=0;//variable i is used to index the number of ports (2 bits) readed from the flit
        
    //this loop reads the flits and puts each port in each position of table.port[i]
    do {
        flit_masked = flit&0xc0000000;
        flit_shifted_masked = flit_shifted&0xc0000000;
        
        table.port[i] = flit_masked >> 30;
        // puts(itoh(table.port[i]));puts("\n");
        
        flit = flit_shifted;
        if((i%16) == 6 && !(flit_masked == EAST  && flit_shifted_masked == WEST  ||//if the path is EW WE SN NS and it should stop here
              flit_masked == WEST  && flit_shifted_masked == EAST  ||
              flit_masked == NORTH && flit_shifted_masked == SOUTH ||
              flit_masked == SOUTH && flit_shifted_masked == NORTH)){//gets a new 32 word bit from NI for i == 7 i == 23 ...
            //flit_shifted = NI_Read_ft(&NI_status);
            flit_shifted = NI_Read();
        }
        else{
            flit_shifted = flit_shifted << 2;
        }
        i++;
        
    }while( !(flit_masked == EAST  && flit_shifted_masked == WEST  ||//if the path is EW WE SN NS and it should stop here
              flit_masked == WEST  && flit_shifted_masked == EAST  ||
              flit_masked == NORTH && flit_shifted_masked == SOUTH ||
              flit_masked == SOUTH && flit_shifted_masked == NORTH ) );
              

    table.port[i] = flit_shifted_masked >> 30;
    //puts(itoh(table.port[i]));puts("\n");
    
    j=i;
    table.path_size = (i/3)+1;
    algorithm = WEST_FIRST;
    for(i=0;i<=j;i++){//calculate the channel to comply with the routing algorithm
        if(algorithm == WEST_FIRST){
            if((table.port[i] == 0x03 || table.port[i] == 0x02) && table.port[i+1] == 0x01){//turns prohibited by WEST FIRST :SW and NW
                algorithm = EAST_FIRST;
            }
        }
        else{
            if((table.port[i] == 0x03 || table.port[i] == 0x02) && table.port[i+1] == 0x0){//turns prohibited by WEST FIRST :SE and NE
                algorithm = WEST_FIRST;
            }
            table.port[i] = table.port[i]+4;
        }
    }
    
    //writes the path as an header of source routing
    shift=24;
    table.path[0] = 0x70007000;
    for(i=0;i<=j;i++){
        table.path[i/6] = table.path[i/6]|((table.port[i]&0x0f) << shift);
        switch(shift){
            case 16:
                shift = 8;
            break;
            case 0:
                shift = 24;
                table.path[i/6+1] = 0x70007000;
            break;
            default:
                shift = shift - 4;
            break;
        }
    }
    
    //broadcast clear
    MemoryWrite(CLEAR_REG,target_seek);
    
    //writes the path in the NI
    MemoryWrite(NI_SEEK,target_seek);//target
    MemoryWrite(NI_SEEK,table.path_size);
    for(i=0;i<=(table.path_size-1)/2;i++){
        MemoryWrite(NI_SEEK,table.path[i]);
        //puts(itoh(table.path[i]));puts("\n");
    }
    
    
    
}

void resend_messages(int remote_addr){
    int i;
    
     char str2[50];
    
    for(i=0; i<PIPE_SIZE; i++) {
			
		if (pipe[i].status == WAITING_ACK && pipe[i].remote_addr == remote_addr) {
            SendCommPacket(remote_addr, MESSAGE_DELIVERY, 0, 0, 0, &pipe[i]);            
            seek_path(remote_addr, 555);
        }
        if (pipe[i].status == WAITING_ACK && pipe[i].local_addr == remote_addr) {
            SendCommPacket(pipe[i].remote_addr, MESSAGE_DELIVERY, 0, 0, 0, &pipe[i]);           
             puts((itoa(MemoryRead(TICK_COUNTER))));
             puts(":SR \n");
             seek_path(pipe[i].remote_addr, 555);
        }

        
    }
}

void resend_messages_xy(int remote_addr){
    int i;
    
     char str2[50];
    
    for(i=0; i<PIPE_SIZE; i++) {
			
		if (pipe[i].status == WAITING_ACK && pipe[i].remote_addr == remote_addr) {
            SendCommPacket(remote_addr, MESSAGE_DELIVERY, 0, 0, 0, &pipe[i]);
            }
        if (pipe[i].status == WAITING_ACK && pipe[i].local_addr == remote_addr) {
            SendCommPacket(pipe[i].remote_addr, MESSAGE_DELIVERY, 0, 0, 0, &pipe[i]);
             puts((itoa(MemoryRead(TICK_COUNTER))));
             puts(":SR \n");
        }
    }
}


/*--------------------------------------------------------------------
* Scheduler - DONE!
*
* DESCRIPTION:
*    Scheduler.
*
*--------------------------------------------------------------------*/
void Scheduler() {

    static unsigned int round_robin = 0;
    unsigned int time_waiting_msg_ack;
    char str[50];
    
    int scheduled = FALSE;
    int i;
    unsigned int avg;

    if (current->status == RUNNING)
    	current->status = READY;
    
    //searches for messages in the pipe waiting for the ack too much time
    for(i=0; i<PIPE_SIZE; i++){
        if (pipe[i].status == WAITING_ACK){
            time_waiting_msg_ack = MemoryRead(TICK_COUNTER) - pipe[i].last_msg_delivery_time;
            
        }
    }
    
    for (i=0; i<MAXLOCALTASKS; i++) {
        if (round_robin == MAXLOCALTASKS-1)
        	round_robin = 0;
        else
        	round_robin++;

        current = &(tcbs[round_robin]);
        
        if (current->status == READY ) {
			/*ready to execute*/
			current->status = RUNNING;
			scheduled = TRUE;

			/*enable timeslice counter*/
			OS_InterruptMaskSet(IRQ_COUNTER18 | IRQ_SEEK | IRQ_RESEND_M);
            
			/*restart timeslice for the task*/
			MemoryWrite(COUNTER_REG, 0);

			break;
        }
    }
    if (!scheduled) {	/* if no ready tasks to execute*/
        current = &idle_tcb;	/* schedules the idle task */
        /*disable timeslice counter*/
        OS_InterruptMaskClear(IRQ_COUNTER18);
    }
}

int add_acks(unsigned int source_cpu, unsigned int target_id, unsigned int source_id, unsigned int sqnc_nbr){
    int i;
    
    //puts("entrei na add_acks\n");
    
    for(i=0;i<LOST_ACKS_SIZE;i++){
        if(lost_acks[i].status == EMPTY){
            lost_acks[i].status = USED;
            lost_acks[i].source_cpu = source_cpu;
            lost_acks[i].source_id = source_id;
            lost_acks[i].target_id = target_id;
            lost_acks[i].sqnc_nbr = sqnc_nbr;
            return 1;
        }
    }
    return 0;
}

int remove_acks(unsigned int remote_addr){
    int i;
    
    //puts("entrei na add_acks\n");
    
    for(i=0;i<LOST_ACKS_SIZE;i++){
        if(lost_acks[i].status == USED && lost_acks[i].source_cpu == remote_addr){
            lost_acks[i].status = EMPTY;
            return 1;
        }
    }
    return 0;
}

/*--------------------------------------------------------------------
* Handler_NI
*
* DESCRIPTION:
*    Initiates the handling of incoming network services.
*
*--------------------------------------------------------------------*/
void Handler_NI() {

    unsigned int i, j, service, task, location;
    

    unsigned int need_scheduling;
	unsigned int source_cpu;
    unsigned int target_id;
    unsigned int source_id;
    unsigned int idle;
    unsigned int code_length;
    unsigned int *offset;
    int pos;
    int NI_status = 0;//used to discover failed reception in NI
    
    unsigned int discarted_msg_length;
    unsigned int time_between_deliver_ack;
    unsigned int sqnc_nbr;
    
    char str[70];
    
    int value;
    int aux_value;
    
    TCB *target;
    PipeSlot *slot_ptr;
    Message *msg;

    need_scheduling = FALSE;

    if (current == &idle_tcb)
    	idle = TRUE;
    else
    	idle = FALSE;


    /* Reads packet target and size together */
    value = NI_Read_ft(&NI_status);
       
    if(NI_status == 0){//there is no error in NI
        if ((value&0xf0000000) == 0x60000000){//backtrack packet
            seek_lock = 0;
            #ifdef SEEK_TIMES
              
            #endif
            receive_seek(value);
            resend_messages(target_seek);
            resend_msg_request(target_seek);
            if (idle){
                need_scheduling = TRUE;
            }
        }
        else{

            /* Reads the packet service */
            service = NI_Read_ft(&NI_status);
            
       
            if(NI_status == 0){//there is no error in NI
                //puts("ok in service from NI\n");
                switch (service) {
                    case MESSAGE_REQUEST:

                   //     puts("MESSAGE_REQUEST\n");
                        
                        /* reads packet source cpu*/
                        source_cpu = NI_Read_ft(&NI_status);

                        /* reads packet message target*/
                        target_id = NI_Read_ft(&NI_status);

                        /* reads packet message source*/
                        source_id = NI_Read_ft(&NI_status);
                        
                        /* Searches for a message from this sorce-target pair which hasn't received and ack*/
                        slot_ptr = GetSlotFromPipe(target_id, source_id, WAITING_ACK);
                        
                        if(slot_ptr){//this message has been received, free the pipe!
                            slot_ptr->status = EMPTY;
                        }
                        else{//this message has not been received 1st message?
                            puts("this message has not been received 1st message?\n");
                        }

                        /* Searches the requested message in the pipe */
                        slot_ptr = GetSlotFromPipe(target_id, source_id, USED);

                        if ( !slot_ptr ) {	/* Message not available yet */

                            // puts("not availabe\n");
                            if ( !InsertMessageRequest(target_id, source_id) ) {	/* Stores the message request */
                                puts("InsertMessageRequest faild.\n");
                                for(;;);
                            }
                        }
                        else {	/* Sends the requested message */
                            //Sends the message
                            #ifdef SERIALCOMM
                                if (there_is_waiting_ack){//there is a message with status WAITING_ACK, wait for send this message
                                    slot_ptr->status = TO_BE_SENT;
                                    slot_ptr->remote_addr = source_cpu;
                                }
                                else{
                                    // puts("msg req\n");
                                    SendCommPacket(source_cpu, MESSAGE_DELIVERY, 0, 0, 0, slot_ptr);

                                    /* Sets the pipe slot which will be transmited by DMA */
                                    last_pipe_slot_sent = slot_ptr;

                                    /* MODIFIED FOR ACK_MESSAGE_DELIVERY */
                                    slot_ptr->status = WAITING_ACK;
                                }
                            #else
                                //sends the requested message
                                SendCommPacket(source_cpu, MESSAGE_DELIVERY, 0, 0, 0, slot_ptr);//source and target id are already in the pipe

                                /* Sets the pipe slot which will be transmited by DMA */
                                last_pipe_slot_sent = slot_ptr;

                                /* MODIFIED FOR ACK_MESSAGE_DELIVERY */
                                slot_ptr->status = WAITING_ACK;
                            #endif
                            
                            /*sets the time i've sent the packet*/
                            slot_ptr->last_msg_delivery_time = MemoryRead(TICK_COUNTER);
                            
                            SearchTerminatedTasks();
                        }
                    break;

                    case MESSAGE_DELIVERY:
   
                        /* Reads the source processor */
                        source_cpu = NI_Read_ft(&NI_status);

                        /* Reads the target task */
                        target_id = NI_Read_ft(&NI_status);

                        /* Reads the source task */
                        source_id = NI_Read_ft(&NI_status);
                        
                        /* Reads the sequence number*/
                        sqnc_nbr = NI_Read_ft(&NI_status);
                                                
                        if(NI_status == 0){//without erros in reception

                            /*remove ack from lost_acks*/
                            // remove_acks(source_cpu);

                            /* Searches the target task TCB */
                            target = SearchTCB(target_id);
                                                       
                            if(last_sqnc_nbr[source_id] >= sqnc_nbr){//if the sequence number is the same - THERE IS error in sqnc number
                                    
                                puts("same sequence #:");
                                puts(itoh(last_sqnc_nbr[source_id]));
                                puts("\n");
                            
                                discarted_msg_length = NI_Read_ft(&NI_status);//then the message should be discarted
                                
                                if(NI_status == 0){//without erros in reception
                                    //processor read because message should be discarted
                                    for(i=0; (i<discarted_msg_length && NI_status == 0); i++)
                                        NI_Read_ft(&NI_status);
                                    
                                    //target_seek=source_cpu;
                                    //start a seek to discover the path
                                    if(seek_lock == 0){
                                        seek_path(source_cpu,2);
                                    }
                                    //stores the source_cpu, source_id and target_id in the lost_acks structure
                                    if(!add_acks(source_cpu, target_id, source_id, sqnc_nbr)){
                                        puts("something went wrong in add_acks\n");
                                    }
                                }
                            }
                            else{//if the sequence number is greater - there is NO error in sqnc number
                                
                                //set the last sqnc nbr after received all packet
                                //last_sqnc_nbr[source_id] = sqnc_nbr;
                                if(!remove_msg_request(source_cpu, target_id, source_id)){
                                    
                                    puts(itoa(MemoryRead(TICK_COUNTER)));
									puts("\n");
									puts("something went wrong with remove msg_request\n");
                                }
                                // sprintf(str,"msg#:%d,src:%d\n\0",sqnc_nbr,source_id);
                                // puts(str);
                                
                                /* Points the target message (ReadPipe argument) */
                                msg = (Message *)(target->offset | target->reg[3]);
                                    
                                /* Reads the message length */
                                msg->length = NI_Read_ft(&NI_status);
                                
                                if(NI_status == 0){
								//	puts("\n");
                                    /* Verifies the DMA availableness */
                                    for(i=0; i<128*2; i++)		/* Waits DMA to send a maximum size message (time out) */
                                        if ( !MemoryRead(DMA_ACTIVE) )
                                            break;

                                    /* After the time out, uses DMA if it is vailable, else processor reads the receiving message */
                                    if ( !MemoryRead(DMA_ACTIVE) ) {
											//puts("!MemoryRead(DMA_ACTIVE)\n");
                                        /* Sets the block size in 32 bits words */
                                        MemoryWrite(DMA_SIZE, msg->length); /* NoC packet size + header (target, paylod size) */

                                        /* Sets the DMA operation (Memory write)*/
                                        MemoryWrite(DMA_OP, WRITE);

                                        /* Sets the block start address */
                                        MemoryWrite(DMA_ADDRESS, (unsigned int)msg->msg);

                                        /* Fires the DMA */
                                        MemoryWrite(DMA_START, 1);

                                        /* Waits the DMA write */
                                        while ( MemoryRead(DMA_ACTIVE) );
                                    }
                                    else	/* Processor read */
                                        for(i=0; i<msg->length; i++)
                                            msg->msg[i] = NI_Read_ft(&NI_status);
                                
                                    NI_status = MemoryRead(FAILED_RCV);
                                
                                    if(NI_status == 0){
                                        target->reg[0] = TRUE; /* return true to ReadPipe()*/
                                        target->status = READY;
                                       // puts("NI_status == 0  return true to ReadPipe\n");
                                        last_sqnc_nbr[source_id] = sqnc_nbr;
                                        //Send ACK_MESSAGE_DELIVERY
                                        //new type of sending message added by wachter
                                        // slot_ptr = GetFreeSlot();
                                        // SendCommPacket(source_cpu, ACK_MESSAGE_DELIVERY, target_id, source_id, sqnc_nbr, slot_ptr);
                                        
                                        if (idle)
                                            need_scheduling = TRUE;
                                    }
                                }
                            }
                        }
                       // puts("MESSAGE_DELIVERY end\n");
                    break;

                    // case ACK_MESSAGE_DELIVERY:
                        
                    //     #ifdef SEEK_TIMES
                    //         // sprintf(str,"ST:%d:ACK\n",MemoryRead(TICK_COUNTER));
                    //         // puts(str);
                    //         puts((itoa(MemoryRead(TICK_COUNTER))));
                    //         puts(":A\n");
                    //     #endif SEEK_TIMES

                    //     /* Reads the target task */
                    //     source_cpu = NI_Read_ft(&NI_status);

                    //     /* Reads the target task */
                    //     target_id = NI_Read_ft(&NI_status);

                    //     /* Reads the source task */
                    //     source_id = NI_Read_ft(&NI_status);
                        
                    //     /* Reads the sequence number */
                    //     sqnc_nbr = NI_Read_ft(&NI_status);
                        
                    //     slot_ptr = GetSlotFromPipe(target_id, source_id, WAITING_ACK);
                                                
                    //     if(!slot_ptr){
                    //         puts("something went wrong searching for the message to ACK\n");
                    //     }
                    //     else{
                    //         if(slot_ptr->order != sqnc_nbr){
                    //             puts("Received ACK from a different sequence number!\n");
                    //             if(seek_lock == 0){
                    //                 seek_path(slot_ptr->remote_addr,3);
                    //             }
                    //         }
                    //         else{
                    //             slot_ptr->status = EMPTY;
                    //             SearchTerminatedTasks();
                    //         }
                    //     }

                    //     add_acks(source_cpu, target_id, source_id, sqnc_nbr);

                    //     #ifdef SERIALCOMM
                    //         if (there_is_waiting_ack) {//there is a
                    //             for(i=0; i<PIPE_SIZE; i++){
                    //                 if (pipe[i].status == TO_BE_SENT){
                    //                     // sprintf(str,"i:%d\n",i);
                    //                     // puts(str);
                    //                     //sends the message with TO_BE_SENT status
                    //                     SendCommPacket(pipe[i].remote_addr, MESSAGE_DELIVERY, 0, 0, 0, &pipe[i]);//source and target id are already in the pipe

                    //                     //set the last pipe slot sent
                    //                     last_pipe_slot_sent = &pipe[i];

                    //                     pipe[i].status = WAITING_ACK;
                    //                     break;
                    //                 }
                    //             }
                    //             if(i==PIPE_SIZE){
                    //                 // puts("zerei!\n");
                    //                 there_is_waiting_ack = 0;
                    //             }
                    //         }
                    //     #endif
                    // break;
                    
                    case TASK_ALLOCATION:
                   
                        /*search for a free TCB*/
                        for(i=0; i<MAXLOCALTASKS; i++)
                            if(tcbs[i].status == FREE)
                                break;

                        tcbs[i].status = USED;

                    // Discard PE_Source
                    //			NI_Read_ft(&NI_status); <<<< NÃO ESQUECER DE DESCOMENTAR QUANDO TASK_ALLOCATION ESTIVER ENVIANDO O PE_SOURCE

                                    /*saves a pointer to the TCB*/
                                    allocatingTCB = &tcbs[i];
                                    allocatingTCB->pc = 0x0;

                                    /* Reads the task id */
                                    allocatingTCB->id = NI_Read_ft(&NI_status);

                                    /* Reads the code length in 32 bits words */
                                    code_length = NI_Read_ft(&NI_status);
                                    
                                            puts("TASK_ALLOCATION\t");				
                                            puts(itoa(allocatingTCB->id));
                                            puts("\t");
                                            puts(itoa(MemoryRead(TICK_COUNTER)));
                                            puts("\n");

                                    /* Verifies the DMA availableness */
                                    for(i=0; i<128*2; i++)		/* Waits DMA to send a maximum size message (time out) */
                                        if ( !MemoryRead(DMA_ACTIVE) )
                                            break;

                                    /* After the time out, uses DMA if it is available, else processor reads the receiving message */
                                    if ( !MemoryRead(DMA_ACTIVE) ) {

                                        /* Sets the block size in 32 bits words */
                                        MemoryWrite(DMA_SIZE, code_length);

                                        /* Sets the DMA operation (Memory write) */
                                        MemoryWrite(DMA_OP, WRITE);

                                        /* Sets the block start address */
                                        MemoryWrite(DMA_ADDRESS, allocatingTCB->offset);

                                        /* Fires the DMA */
                                        MemoryWrite(DMA_START, 1);

                                        /* Waits the DMA write */
                                        while ( MemoryRead(DMA_ACTIVE) );
                                    }
                                    else {	/* Processor read */

                                        /* Points the page begin */
                                        offset = (unsigned int*)allocatingTCB->offset;

                                        for(i=0; i<code_length; i++)
                                            offset[i] = NI_Read_ft(&NI_status);
                                    }

                                    allocatingTCB->status = READY;

                                    task_location[allocatingTCB->id].task = allocatingTCB->id;
                                    task_location[allocatingTCB->id].processor = net_address;
                                    
                                    request_task[allocatingTCB->id] = -1;
                                    location_request_task[allocatingTCB->id] = -1;
                                    
                                    if ( idle )
                                        need_scheduling = TRUE;

                    break;

                    case TASK_ALLOCATED:
  
                        // Discard PE_Source
                        NI_Read_ft(&NI_status);

                        /* Reads the processor location */
                        location = NI_Read_ft(&NI_status);

                        /* Reads the task id */
                        task = NI_Read_ft(&NI_status);

                        /* Verifies if the task location is already stored */
                        if ( task_location[task].task != task ){
                        
                            task_location[task].task = task;
                            task_location[task].processor = location;
                            request_task[task] = -1;
                            
                            /* Removes the location task request, if any */
                            location_request_task[task] = -1;
                            
                            /* Searches for a message request stored from 'task' */
                            source_id = SearchRequestingMessage(task);

                            if ( source_id != - 1 ) {	/* Verifies if threre is a stored message request from 'task' */

                                /*** Sends a stored message to 'task', if any ***/
                                if ( (slot_ptr = GetSlotFromPipe(task, source_id, USED)) ) {

                                    /* Removes the message request */
                                    GetRequestingMessage(source_id, task);

                                    #ifdef SERIALCOMM
                                        if (there_is_waiting_ack){//there is a message with status WAITING_ACK, wait for send this message
                                            slot_ptr->status = TO_BE_SENT;
                                            slot_ptr->remote_addr = location;
                                        }
                                        else{
                                            // puts("task allocated\n");
                                            SendCommPacket(location, MESSAGE_DELIVERY, 0, 0, 0, slot_ptr);
                                            
                                            /* Sets the pipe slot which will be transmited by DMA */
                                            last_pipe_slot_sent = slot_ptr;

                                            /* MODIFIED FOR ACK_MESSAGE_DELIVERY */
                                            slot_ptr->status = WAITING_ACK;
                                        }
                                    #else
                                        SendCommPacket(location, MESSAGE_DELIVERY, 0, 0, 0, slot_ptr);
                                        
                                        /* Sets the pipe slot which will be transmited by DMA */
                                        last_pipe_slot_sent = slot_ptr;

                                        /* MODIFIED FOR ACK_MESSAGE_DELIVERY */
                                        slot_ptr->status = WAITING_ACK;
                                    #endif
                                                                        
                                    /*sets the time i've sent the packet*/
                                    slot_ptr->last_msg_delivery_time = MemoryRead(TICK_COUNTER);

                                    /* Searches for TERMINATED tasks */
                                    SearchTerminatedTasks();
                                }
                            }
                        }
                    break;

                    case TASK_DEALLOCATED:
                    
                        /*reads source processor - but this information is not used*/
                        location = NI_Read_ft(&NI_status);

                        /*reads task id*/
                        task = NI_Read_ft(&NI_status);
                        
                        task_location[task].task = NOT_ALLOCATED;
                        request_task[task] = -1;
                        location_request_task[task] = -1;

                    
                    break;

                    case LOCATION_ANSWER:

                        // Discard PE_Source
                        NI_Read_ft(&NI_status);

                        /* Reads the requesting task */
                        //target_id = NI_Read_ft(&NI_status);

                        /* Reads the requested task */
                        source_id = NI_Read_ft(&NI_status);

                        /* Reads the requested task location */
                        source_cpu = NI_Read_ft(&NI_status);

                        pos = -1;
                    
                        task_location[source_id].task = source_id;
                        task_location[source_id].processor = source_cpu;
                        
                        request_task[source_id] = -1;
                        location_request_task[source_id] = -1;

                    break;

                    default:
						puts("ERROR: SERVICE UNKNOWN!\n");
                        sprintf(str,"Service:%x\n",service);
                    break;
                }
                
            }
            else{
                puts("error in service from NI - discard packet\n");
            }
        }
    }
    else{
        puts("error in first word from NI - discard packet\n");
    }
    
    if(NI_status == 1){
		puts(" service --- error in NI - discard packet\n");
		MemoryWrite(RELEASE_NI,1);
    }

    if (need_scheduling) /* runs the scheduling algorithm.*/
        Scheduler();
}

/*--------------------------------------------------------------------
* OS_Init
*
* DESCRIPTION:
*    Initializes system's variables.
*
*--------------------------------------------------------------------*/
void OS_Init() {

	int i, j;
	#ifdef PLASMA
    int *p;
    
    p = (int *)4;
    idle_tcb.reg[24] = *p & 0x0000FFFF;  //kernel global pointer
    gp_ptr = &(idle_tcb.reg[24]);
    
    p = (int *)12;
    idle_tcb.reg[25] = *p & 0x0000FFFF;  //kernel stack pointer
    sp_ptr = &(idle_tcb.reg[25]);
	#endif
	
	#ifdef MBLITE
	sp_ptr = &(idle_tcb.reg[8]);
	 __asm__ volatile (	"la r18, r0, sp_ptr;"/*      define o endereço que aponta para o stack pointer */
					"lw r18, r18, r0;"    /*    lê o endereço do stack pointer */
					"swi r1, r18, 0;"); /* grava o r1 na variavel sp_ptr*/
    #endif

    idle_tcb.pc = (unsigned int) &OS_Idle;
    idle_tcb.id = 0;
    idle_tcb.offset = 0;
    idle_tcb.status = READY;
    current = &idle_tcb;

    net_address = MemoryRead(NI_CONFIG);
    
    /*cleanup
    init pipe occupation*/
    for(j=0; j<PIPE_SIZE; j++) {
    	pipe[j].status = EMPTY;  /*all positions set to EMPTY*/
    	pipe[j].last_msg_delivery_time = 100000;  /*all threshold set to zero*/
		pipe[j].local_addr = net_address;	/*Sets the local processor address (constant)*/
	}


   /* Sets the temporary slots as available and the constant local processor address */
   slot1.status = EMPTY;
   slot1.local_addr = net_address;
   slot2.status = EMPTY;
   slot2.local_addr = net_address;

   last_pipe_slot_sent = 0;


	#ifdef MBLITE
    /*init tcbs*/
    for(i=0; i<MAXLOCALTASKS; i++) {
        tcbs[i].status = FREE;
        tcbs[i].reg[14] = 0;
        tcbs[i].reg[8] = *sp_ptr;
		tcbs[i].offset = PAGESIZE*(i + KERNELPAGECOUNT);
    }
	#endif
	
	#ifdef PLASMA
	for(i=0; i<MAXLOCALTASKS; i++) {
		tcbs[i].status = FREE;
		tcbs[i].pc = 0;
		tcbs[i].offset = PAGESIZE*(i + KERNELPAGECOUNT);
    }
	#endif


    /*initializes all entries for the requestMessage table*/
    for(i = 0; i < REQUEST_SIZE; i++)
    	requestMessage[i].requesting = -1;
        
    //NEEDED BY ACK_MESSAGE_DELIVERY
    for(i=0;i<LOST_ACKS_SIZE;i++){
        lost_acks[i].status = EMPTY;/*all status set to empty*/
    }
    for(i=0;i<MSG_REQUEST_SIZE;i++){
        msg_request[i].status = EMPTY;/*all status set to empty*/
    	msg_request[i].last_msg_request_time = 0;  /*all threshold set to zero*/
    }
    
    //resets the timers for ft propouses
    for(i=0; i<MAX_GLOBAL_TASKS; i++){
        last_sqnc_nbr[i] = 0;
        //avg_time_between_deliver_ack[i] = 100000;
        //avg_time_between_req_deliver[i] = 100000;
        
        // total_time_between_req_deliver[i] = 0;
        // total_time_between_deliver_ack[i] = 0;

        // total_nbr_req_deliver[i] = 0;
        // total_nbr_deliver_ack[i] = 0;
        // total_nbr_deliver_ack[i] = 0;
        
    }
    
    seek_lock = 0;
    last_print = 0;
    //sqnc_nbr = 0;
    
    //puts("finish OS_Init()\n");

	#ifdef PLASMA
	/*disable interrupts*/
    OS_InterruptMaskClear(0xffffffff);

    /*enables timeslice counter and wrapper interrupts*/
    OS_InterruptMaskSet(IRQ_COUNTER18 | IRQ_NOC | IRQ_SEEK | IRQ_RESEND_M);
	#endif
	#ifdef MBLITE
    OS_Idle();
	#endif
    
}


#ifdef MBLITE
/*--------------------------------------------------------------------
* ASM_RunScheduledTask
*
* DESCRIPTION:
*    Recover the task context.
*
*--------------------------------------------------------------------*/
int reg;
void ASM_RunScheduledTask(){
	__asm__ volatile (	"la r18, r0, current;"
						"lw r18, r0, r18;"
						"lwi r3, r18, 0 ;"
						"lwi r4, r18, 4 ;"
						"lwi r5, r18, 8 ;"
						"lwi r6, r18, 12;"
						"lwi r7, r18, 16;"
						"lwi r8, r18, 20;"
						"lwi r9, r18, 24;"
						"lwi r10,r18, 28;"
						"lwi r1, r18, 32;"
						"lwi r2, r18, 36;"
						"lwi r11, r18, 40;"
						"lwi r12, r18, 44;"
						"lwi r13, r18, 48;"
						"lwi r14, r18, 52;"
						"lwi r15, r18, 56;"
						"lwi r16, r18, 60;");
	__asm__ volatile (	"lwi r17, r18, 64;"
						
						"lwi r19, r18, 68;"
						
						"lwi r21, r18, 76;"
						"lwi r22, r18, 80;"
						"lwi r23, r18, 84;"
						"lwi r24, r18, 88;"
						"lwi r25, r18, 92;"
						"lwi r26, r18, 96;"
						"lwi r27, r18, 100;"
						"lwi r28, r18, 104;"
						"lwi r29, r18, 108;"
						"lwi r30, r18, 112;"
						"lwi r31, r18, 116;"
						"lwi r18, r0, reg;"
						"or	r0, r0, r0;"
						"rtid	r14, 0;"
						"or	r0, r0, r0;");
}
#endif

/*--------------------------------------------------------------------
* OS_InterruptServiceRoutine - DONE!
*
* DESCRIPTION:
*    Executes the registered interrupt handler.
*
*--------------------------------------------------------------------*/
#ifdef MBLITE
int dummy;
int OS_InterruptServiceRoutine(unsigned int service, unsigned int arg0, unsigned int arg1, unsigned int arg2) {
	int status;
	/*SET THE STACK POINTER OF THE OS*/
	sp_ptr = &(idle_tcb.reg[8]);
	__asm__ volatile (	"la  r18, r0, sp_ptr;"
						"lw r18, r0, r18;"
						"lwi r1, r18, 0;");
	
	status = MemoryRead(IRQ_STATUS);
	
	if ( status & IRQ_SYS_CALL ){
		MemoryWrite(SYS_CALL,FALSE);/*unset syscall interrupt*/
		/*dummy = Syscall(current->reg[2],current->reg[3],current->reg[4],current->reg[5]);*/
		dummy = Syscall(service, arg0, arg1, arg2);
		__asm__ volatile (	"la r18, r0, current;"/*      define o endereço que aponta para a TCB atual */
							"lw r18, r0, r18;"    /*    lê o endereço da TCB ATUAL */
							"swi r3, r18, 0;"); /* grava o retorno (r3 =variavel dummy) da funcao sys_call*/
		if (bug_syscall == TRUE){
			Scheduler();
		}
	}
	else{
		if( status & IRQ_NOC ){
			Handler_NI();
		}
		else{
			if ( status & IRQ_COUNTER18 ){
				Scheduler();
			}
		}
	}
	
	MemoryWrite(NEXT_PAGE,current->offset);
	ASM_RunScheduledTask();
}
#endif

#ifdef PLASMA
void OS_InterruptServiceRoutine(unsigned int status) {
//puts("status\n");
int i;
i=0;
 //puts(itoa(status));
 //puts("\n");
    if ( status & IRQ_COUNTER18 )
    	Scheduler();
    else if ( status & IRQ_NOC )
    	Handler_NI();
	else if (status & IRQ_SEEK){
		puts("status & IRQ_SEEK\n");
		  SeekUnreachable();
        }
    else if (status & IRQ_RESEND_M){
			SeekResend();
				}	
	
    /*runs the scheduled task*/
    ASM_RunScheduledTask(current);
}
#endif

/*--------------------------------------------------------------------
* OS_InterruptMaskClear
*
* DESCRIPTION:
*    Disables selected interrupts.
*    Plasma hardware dependent
*
*--------------------------------------------------------------------*/
unsigned int OS_InterruptMaskClear(unsigned int Mask) {

    unsigned int mask;

    mask = MemoryRead(IRQ_MASK) & ~Mask;
    MemoryWrite(IRQ_MASK, mask);

    return mask;
}

/*--------------------------------------------------------------------
* OS_InterruptMaskSet
*
* DESCRIPTION:
*    Enables selected interrupts.
*    Plasma hardware dependent
*
*--------------------------------------------------------------------*/
unsigned int OS_InterruptMaskSet(unsigned int Mask) {

    unsigned int mask;

    mask = MemoryRead(IRQ_MASK) | Mask;
    MemoryWrite(IRQ_MASK, mask);

    return mask;
}

/*--------------------------------------------------------------------
* OS_Idle
*
* DESCRIPTION:
*    Runs when there's no tasks ready.
*--------------------------------------------------------------------*/
#ifdef MBLITE
void OS_Idle() {
	OS_InterruptMaskSet(0xffffffff);
	for(;;){
		MemoryWrite(0x25555555,0xF0DAEB0A);
	}
}

#else /*PLASMA*/
void OS_Idle() {
    unsigned int i;
    unsigned int time_waiting_msg_request;
    unsigned int time_waiting_msg_ack;
    unsigned int avg;
    unsigned int now;
    int search_over = 0;
    char str[50];
    
	for (;;){
         }
}
#endif

/*--------------------------------------------------------------------
* Main
*
* DESCRIPTION:
*    Main.
*--------------------------------------------------------------------*/


int main(){
    int i;
	#ifdef PLASMA
	ASM_SetInterruptEnable(FALSE);
	#endif
	
	OS_Init();
    
      
	#ifdef MBLITE
	OS_Idle();
	#endif
	
	#ifdef PLASMA
	/*runs the scheduled task*/
    ASM_RunScheduledTask(current);
	#endif
	
	return 0;
}
