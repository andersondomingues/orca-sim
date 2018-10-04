/*--------------------------------------------------------------------
 * TITLE: Plasma microkernel
 * AUTHOR: Cristiane Raquel Woszezenki (cristianew@inf.pucrs.br)
 *         Ismael Grehs (grehs@inf.pucrs.br)
 * DATE CREATED: 23/05/06
 * FILENAME: task.h
 * PROJECT: Plasma CPU core
 *
 * DESCRIPTION:
 *    Tasks definitions
 *--------------------------------------------------------------------*/

#ifndef __TASK_H__
#define __TASK_H__

// Syscalls
#define EXIT      0
#define WRITEPIPE 1
#define READPIPE  2
#define GETTICK   3
#define ECHO      4

extern int SystemCall();	/* Defined in bootTask.asm */

#define Send(msg, target) while(!SystemCall(WRITEPIPE, (unsigned int*)msg, target))
#define Receive(msg, source) while(!SystemCall(READPIPE, (unsigned int*)msg, source))
#define GetTick() SystemCall(GETTICK)
#define Echo(str) SystemCall(ECHO, (char*)str)

/*--------------------------------------------------------------------
 * struct Message
 *
 * DESCRIPTION:
 *    Used to handle messages inside the task.
 *    This is not the same structure used in the kernels.
 *
 *--------------------------------------------------------------------*/
#define MSG_SIZE 128

typedef struct {
	int length;
	int msg[MSG_SIZE];
} Message;

#endif //__TASK_H__
