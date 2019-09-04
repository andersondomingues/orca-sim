#ifndef _NOC_TEST4_H
#define _NOC_TEST4_H

#include <hellfire.h>
#include <noc.h>


//#define TASK_ATTRIBUTE __attribute__ ((used))

void noc_test4_sender(void)		__attribute__((section (".tasks")));
void noc_test4_receiver (void)	__attribute__((section (".tasks")));

#endif /* _NOC_TEST4_H */