/*--------------------------------------------------------------------------------
 * TITLE: MBLite and Plasma V3 microkernel
 * AUTHOR: CARLO LUCAS
 * DATE CREATED: 12/04/2011
 * FILENAME: kernelV3.h
 * PROJECT: Hemps-S
 * DESCRIPTION: 
 *          Microkernel for Plasma and MBLite processors
 *          Based in the microkernel kernel_V2_MBlite.c for the MBLite processor
 *          and in the microkernel kernel.c (V2_Plasma) for the Plasma processor
 *--------------------------------------------------------------------------------*/

#ifndef __SEEK_H__
#define __SEEK_H__

//definition of the number of times that one task should enter in the Scheduler waiting for a ack_delivery
#define MAX_THRESHOLD   0x20
#define MAX_PATH_SIZE   0x20
#define EAST            0x00000000
#define WEST            0x40000000
#define NORTH           0x80000000
#define SOUTH           0xc0000000
#define WEST_FIRST      0
#define EAST_FIRST      1


typedef struct {
    unsigned int tableSlotStatus;
    unsigned int target;
    unsigned int path_size;
    unsigned int path[MAX_PATH_SIZE];
    unsigned char port[64];
    //unsigned char channel[64];
} tableSlot;


//enum lost_ackSlotStatus {EMPTY, USED};

//#####WARNING, ASSUMING 2*MAXLOCALTASKS as maximum size
#define LOST_ACKS_SIZE  2*MAXLOCALTASKS

typedef struct {
    enum PipeSlotStatus status;
    unsigned int source_cpu;
    unsigned int source_id;
    unsigned int target_id;
    unsigned int sqnc_nbr;
} lost_ackSlot;

#define MSG_REQUEST_SIZE  2*MAXLOCALTASKS
typedef struct {
    enum PipeSlotStatus status;
    unsigned int processor;
    unsigned int source_id;
    unsigned int target_id;
    unsigned int last_msg_request_time;
} msg_requestSlot;


#endif
