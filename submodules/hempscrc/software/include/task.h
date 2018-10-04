/*--------------------------------------------------------------------
 * TITLE: Plasma microkernel - ported to mb-lite
 * AUTHOR: Cristiane Raquel Woszezenki (cristianew@inf.pucrs.br)
 *         Ismael Grehs (grehs@inf.pucrs.br)
 *         Eduardo Wachter (eduardo.wachter@gmail.com)
 * DATE CREATED: 18/10/2010
 * FILENAME: task.h
 * PROJECT: Plasma CPU core
 *
 * DESCRIPTION:
 *    Tasks definitions
 *--------------------------------------------------------------------*/

#ifndef __TASK_H__
#define __TASK_H__

/* Syscalls*/
#define EXIT      0
#define WRITEPIPE 1
#define READPIPE  2
#define GETTICK   3
#define ECHO      4
#define SEEK      5

#define MemoryWrite(A,V) *(volatile unsigned int*)(A)=(V)
#define TRUE	1
#define FALSE	0

#ifdef PLASMA
extern int SystemCall();
#else
/*SYS_CALL*/
#define SYS_CALL		0x20000070
int SystemCall(int service, unsigned int arg0, unsigned int arg1, unsigned int arg2){
	MemoryWrite(SYS_CALL,TRUE);
	return 0;
}
#endif

#define Send(msg, target) while(!SystemCall(WRITEPIPE, (unsigned int*)msg, target,0))
#define Receive(msg, source) while(!SystemCall(READPIPE, (unsigned int*)msg, source,0))
#define GetTick() SystemCall(GETTICK,0,0,0)
#define Echo(str) SystemCall(ECHO, (char*)str,0,0)
#define exit() SystemCall(EXIT, 0, 0, 0)
//#define seek_req(target) SystemCall(SEEK, target, 0, 0)

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

#endif /*__TASK_H__*/

