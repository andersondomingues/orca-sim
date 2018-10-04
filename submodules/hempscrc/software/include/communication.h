/*--------------------------------------------------------------------
 * TITLE: HeMPS communication packets
 * AUTHOR: Eduardo Wachter (eduardo.wachte@acad.pucrs.br)
 * DATE CREATED: 12/04/13
 * FILENAME: communication.h
 * PROJECT: HeMPS
 *
 * DESCRIPTION:
 *    HeMPS communication functions
 *    HeMPS communication packet struct
 *--------------------------------------------------------------------*/

#ifndef __COMM_H__
#define __COMM_H__

#include "task.h"
#include "plasma.h"
//#include "../kernel/master/kernel.h"

#define SEEK_TIMES

//**********************************
//**********************************
//**********************************
//**********************************
//variable used by Kernel master
//**********************************
//**********************************
//**********************************
//**********************************

 typedef struct {
     int task;
     int flits;
 } dependence;

 typedef struct {
     int type;
     char dependences;
     dependence taskDep[10];
 } tsk;

 /* Points the task object code in the tasks repository */
 typedef struct {
    int id;
    int size;
    int proc_addr;
    int *address;
    dependence dependences[10];
 } TaskPackage;

unsigned int there_is_waiting_ack;

//**********************************
//**********************************
//**********************************
//**********************************
// variable used by Kernel Slave
//**********************************
//**********************************
//**********************************
//**********************************

 unsigned int net_address;           /*local processor's noc address */

/*--------------------------------------------------------------------
 * struct PipeSlot
 *
 * DESCRIPTION:
 *    Memory image of the NoC packet. Transmited by DMA.
 *
 *--------------------------------------------------------------------*/
enum PipeSlotStatus {EMPTY, USED, WAITING_ACK, TO_BE_SENT};

typedef struct {
    //HEADER
    unsigned int remote_addr;       /* Remote processor address */
    unsigned int pkt_size;          /* NoC packet size (flits) */
    unsigned int service;           /* Service identifier */
    unsigned int local_addr;        /* Local processor address */
    //PAYLOAD
    unsigned int target;            /* Target task */
    unsigned int source;            /* Source task */
    unsigned int order;
    unsigned int length;            /* Message lenght (32 bits words)*/
    unsigned int message[MSG_SIZE];
    enum PipeSlotStatus status;
    //unsigned int order;
    unsigned int last_msg_delivery_time;
} PipeSlot;

#define PIPE_SIZE       12
PipeSlot pipe[PIPE_SIZE];           /*global pipe*/

typedef struct {
    //HEADER
    unsigned int pe_target;             /* target processor address : 1 flit */
    unsigned int pkt_size;              /* NoC packet size (flits) : 1 flit */
    unsigned int service;               /* Service identifier : 2 flit */
    unsigned int pe_source;             /* source processor address : 2 flit */

    //PAYLOAD
    unsigned int payload_field[2];      /* Time the packet is inserted : 2 flit */
} ctrl_packet;

/*--------------------------------------------------------------------
* DMA_Send
*
* DESCRIPTION:
*    Programates the DMA to transmit a memory block through the NoC using the network interface.
*
*--------------------------------------------------------------------*/
void DMA_Send(PipeSlot* slot) {

    /* Waits the DMA availableness */
    while ( MemoryRead(DMA_ACTIVE) );

    /* Sets the block size in 32 bits words */
    MemoryWrite(DMA_SIZE, (slot->pkt_size>>1) + 2); /* NoC packet size + header (target, paylod size) */

    /* Sets the DMA operation (Memory read) */
    MemoryWrite(DMA_OP, READ);

    /* Sets the block start address */
    MemoryWrite(DMA_ADDRESS, (unsigned int)slot);

    /* Fires the DMA */
    MemoryWrite(DMA_START, 1);
    
#ifdef MBLITE
        /* Waits the DMA write */
    while ( MemoryRead(DMA_ACTIVE) );
#endif
}

void DmaSendCtrl(ctrl_packet* packet) {

    /* Waits the DMA availableness */
    while ( MemoryRead(DMA_ACTIVE) );

    /* Sets the block size in 32 bits words */
    MemoryWrite(DMA_SIZE, (packet->pkt_size>>1) + 2); /* NoC packet size + header (target, paylod size) */

    /* Sets the DMA operation (Memory read) */
    MemoryWrite(DMA_OP, READ);

    /* Sets the block start address */
    MemoryWrite(DMA_ADDRESS, (unsigned int)packet);

    /* Fires the DMA */
    MemoryWrite(DMA_START, 1);
    
#ifdef MBLITE
        /* Waits the DMA write */
    while ( MemoryRead(DMA_ACTIVE) );
#endif
}


int SendCommPacket(unsigned int pe_target, unsigned int service, unsigned int task_id_target, unsigned int task_id_source, unsigned int sqnc_nbr, PipeSlot *pipeslot){
    
    int status;
    char str[20];
    int utilization,i;
    
    pipeslot->remote_addr = pe_target;
    
    pipeslot->service = service;

    //pipeslot->timestamp = MemoryRead(TICK_COUNTER);

    switch(pipeslot->service){
        case MESSAGE_DELIVERY:
         	pipeslot->pkt_size = (pipeslot->length << 1) + 12;
            there_is_waiting_ack = 1;
            DMA_Send(pipeslot);
            #ifdef SEEK_TIMES
                // sprintf(str,"ST:%d:DEL\n",MemoryRead(TICK_COUNTER));
                // puts(str);
                puts((itoa(MemoryRead(TICK_COUNTER))));
                puts(":D\n");
            #endif
            status = 1;
            // utilization=0;
            // for(i=0; i<PIPE_SIZE; i++) {
            //     if(pipe[i].status == USED || pipe[i].status == WAITING_ACK || pipe[i].status == TO_BE_SENT){
            //         utilization++;
            //     }
            // }
            // sprintf(str,"pipe_ut:%d\n",utilization);
            // puts(str);

        break;

        // case ACK_MESSAGE_DELIVERY:
        //     pipeslot->target = task_id_target;

        //     pipeslot->source = task_id_source;

        //     pipeslot->order = sqnc_nbr;

        // 	pipeslot->pkt_size = 10;
        //     DMA_Send(pipeslot);
        //     status = 1;
        // break;

        case MESSAGE_REQUEST:

            pipeslot->target = task_id_target;

            pipeslot->source = task_id_source;

            pipeslot->pkt_size = 8;
            DMA_Send(pipeslot);
            status = 1;
        break;

        default:
            puts("unknown communication service\n");
            status = -1;
        break;
    }

    return status; 
}

int SendCtrlPacket(unsigned int pe_target, unsigned int service, unsigned int field0, unsigned int field1, TaskPackage *task){
    ctrl_packet packet;
    int status;

    packet.pe_target = pe_target;

    packet.service = service;

    packet.pe_source = net_address;

    switch(packet.service){
        case TASK_ALLOCATED:
            packet.payload_field[0] = field0;//proc
            packet.payload_field[1] = field1;//taskID
            packet.pkt_size = 8;

            DmaSendCtrl(&packet);
            status = 1;
    	break;

        case TASK_TERMINATED:
        case TASK_DEALLOCATED:
            packet.payload_field[0] = field0;//task_id
            packet.pkt_size = 6;

            DmaSendCtrl(&packet);
            status = 1;
        break;


        case TASK_REQUEST:
        case LOCATION_REQUEST:
    	case LOCATION_ANSWER:
            packet.payload_field[0] = field0;//target_id
            packet.payload_field[1] = field1;//source_id
            packet.pkt_size = 8;

            DmaSendCtrl(&packet);
            status = 1;
        break;
        
    	case TASK_ALLOCATION:
            //task_pes[task->id & 0x7fffffff] = pe_target;
    	
    		/* Header injection */
    		/* Sends the target slave */
    		MemoryWrite(DMA_TARGET,pe_target);

    		/* Sends the NoC packet size (flit) */
    		MemoryWrite(DMA_SIZE, task->size*2 + 6);

    		/* Sends the task id */
    		MemoryWrite(DMA_TASK_ID,task->id & 0x7fffffff);

    		/* Sets the DMA operation (Memory read) */
    		MemoryWrite(DMA_OP, READ);

    		/* Sets the block start address */
    		MemoryWrite(DMA_ADDRESS, 0x10000000 | (unsigned int)task->address);

    		/* Fires the DMA */
    		MemoryWrite(DMA_START, 1);
            
            status = 1;
        break;

        default:
            puts("unknown control service\n");
            status = -1;
        break;
    }

    return status;
}

#endif
