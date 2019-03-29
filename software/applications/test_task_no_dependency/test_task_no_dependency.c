#include "test_task_no_dependency.h"

#define BUFFER_SIZE 20

void test_task_no_dependency(void)
{
	unsigned char i, j;
	
	printf("Anderson");
	
	j = 0;
	while(j < 1000) {
    	i = 0;
    	while(i < 1000){
        	asm("add x0 x0");
			asm("addi a2,zero,4");
			asm("addi a1,a1,-4");
		}
		
        printf("lalal %d %d", j, i);
	}
}