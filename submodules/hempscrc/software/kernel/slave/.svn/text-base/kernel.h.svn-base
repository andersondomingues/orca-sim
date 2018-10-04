/*--------------------------------------------------------------------------------
 * TITLE: MBLite and Plasma V3 microkernel
 * AUTHOR: CARLO LUCAS
 * DATE CREATED: 12/04/2011
 * FILENAME: kernelV3.h
 * PROJECT: Hemps-S
 * DESCRIPTION: 
 * 			Microkernel for Plasma and MBLite processors
 * 			Based in the microkernel kernel_V2_MBlite.c for the MBLite processor
 * 			and in the microkernel kernel.c (V2_Plasma) for the Plasma processor
 *--------------------------------------------------------------------------------*/

#ifndef __KERNEL_H__
#define __KERNEL_H__

//#define MSG_SIZE 128
/*---------------------------------------------------------------------------------
 * INCLUDE
 * -------------------------------------------------------------------------------*/

#include "../../include/task.h"		/* struct Message, MSG_SIZE and syscalls */

/*---------------------------------------------------------------------------------
 *  PARAMETERS DEFINES
 * -------------------------------------------------------------------------------*/
#define REQUEST_SIZE	12

/*--------------------------------------------------------------------------------
 * TASKS STATUS
 * -------------------------------------------------------------------------------*/
#define READY       	1
#define WAITING     	2
#define TERMINATED  	3
#define RUNNING     	4
#define FREE        	5
#define ALLOCATING  	6
#define NOT_ALLOCATED  -1

/*--------------------------------------------------------------------
 * struct TCB
 *
 * DESCRIPTION:
 *    Task control block.
 *--------------------------------------------------------------------*/
#ifdef MBLITE
typedef struct {
    unsigned int reg[31];       /*30 registers (Vn,An,Tn,Sn,RA)*/
    unsigned int pc;            /*program counter*/
    unsigned int offset;        /*initial address of the code*/
    unsigned int id;            /*identifier*/
    unsigned int status;        /*status (READY, WAITING, TERMINATED, RUNNING, FREE, ALLOCATING)*/
} TCB;

#else /*PLASMA*/
typedef struct {
    unsigned int reg[30];       /*30 registers (Vn,An,Tn,Sn,RA)*/
    unsigned int pc;            /*program counter*/
    unsigned int offset;        /*initial address of the code*/
    unsigned int id;            /*identifier*/
    unsigned int status;        /*status (READY, WAITING, TERMINATED, RUNNING, FREE, ALLOCATING)*/
} TCB;
#endif

/*--------------------------------------------------------------------
 * struct RequestMessage
 *
 * DESCRIPTION:
 *    Stores a request for one message.
 *
 *--------------------------------------------------------------------*/
typedef struct {
    int requesting;             /*task that is requesting the message (target)*/
    int requested;              /*task source*/
} RequestMessage;

#ifdef PLASMA

extern unsigned int ASM_SetInterruptEnable(unsigned int);
extern void ASM_SaveRemainingContext(TCB*);
extern void ASM_RunScheduledTask(TCB*);

void OS_InterruptServiceRoutine(unsigned int);
#endif
/* ISR*/
unsigned int OS_InterruptMaskSet(unsigned int);
unsigned int OS_InterruptMaskClear(unsigned int);


/* Init*/
void OS_Init();
void OS_Idle();

/* Scheduler*/
void Scheduler();

#endif
