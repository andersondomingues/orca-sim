/*--------------------------------------------------------------------
 * TITLE: Plasma microkernel
 * AUTHOR: Cristiane Raquel Woszezenki (cristianew@inf.pucrs.br)
 *         Ismael Augusto Grehs (grehs@inf.pucrs.br)
 * DATE CREATED: 19/04/06
 * FILENAME: kernel.h
 * PROJECT: MPSoC Plasma
 * DESCRIPTION:
 *    Plasma microkernel
 *--------------------------------------------------------------------*/
#ifndef __KERNEL_H__
#define __KERNEL_H__

/* Indicates an incoming service */
#define NoC_INT		(MemoryRead(IRQ_STATUS) & IRQ_NOC)
#define	Task_Req (MemoryRead(REQ_TASK) & 0x80000000)
#define	External_Task (MemoryRead(REQ_TASK) & 0x7fffffff)

#define NOT_ALLOCATED	-1
#define NONE			-1
#define DYNAMIC			-1
#define TERMINATED		-2

#endif
